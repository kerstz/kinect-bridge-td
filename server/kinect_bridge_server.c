// kinect_bridge_server.c — v2 : registered depth(mm), IR, record/playback, LED/tilt
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <errno.h>
#include <libfreenect.h>

static int g_use_ir = 0;
static FILE* g_rec = NULL;
static pthread_mutex_t g_rec_mtx = PTHREAD_MUTEX_INITIALIZER;
static uint64_t g_last_ts = 0;
static volatile int g_req_video = 0;
static volatile int g_cur_ir = -1;

typedef struct { uint8_t* data; uint32_t len, w, h, fmt; uint64_t counter; } Slot;
static Slot g_video, g_depth;
static pthread_mutex_t g_vid_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_dep_mtx = PTHREAD_MUTEX_INITIALIZER;
static freenect_context* g_ctx = NULL;
static freenect_device*  g_dev = NULL;

static uint64_t now_ms(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000; }

static void feed_frame(int is_depth, uint32_t w, uint32_t h, uint32_t fmt, const void* data, uint32_t len){
    Slot* s = is_depth? &g_depth : &g_video;
    pthread_mutex_t* m = is_depth? &g_dep_mtx : &g_vid_mtx;
    pthread_mutex_lock(m);
    if (s->len < len){ free(s->data); s->data = (uint8_t*)malloc(len); }
    memcpy(s->data, data, len);
    s->len=len; s->w=w; s->h=h; s->fmt=fmt; s->counter++;
    pthread_mutex_unlock(m);
    if (g_rec){
        pthread_mutex_lock(&g_rec_mtx);
        uint32_t hdr[6];
        uint64_t t=now_ms(); uint32_t dt = g_last_ts ? (uint32_t)(t-g_last_ts) : 33; g_last_ts=t;
        hdr[0]=htonl(is_depth?1:0); hdr[1]=htonl(dt);
        hdr[2]=htonl(0x4B494E31u); hdr[3]=htonl(w); hdr[4]=htonl(h); hdr[5]=htonl(fmt);
        fwrite(hdr,sizeof(hdr),1,g_rec); fwrite(data,1,len,g_rec); fflush(g_rec);
        pthread_mutex_unlock(&g_rec_mtx);
    }
}

static void depth_cb(freenect_device* d, void* data, uint32_t ts){
    freenect_frame_mode m = freenect_get_current_depth_mode(d);
    feed_frame(1, m.width, m.height, 1, data, m.bytes);
}
static void video_cb(freenect_device* d, void* data, uint32_t ts){
    freenect_frame_mode m = freenect_get_current_video_mode(d);
    feed_frame(0, m.width, m.height, (g_cur_ir==1)?2:0, data, m.bytes);
}

static int send_all(int fd, const void* b, uint32_t len){
    const char* p=(const char*)b; uint32_t got=0;
    while(got<len){ ssize_t n=send(fd,p+got,len-got,0); if(n<=0)return -1; got+=n; }
    return 0;
}

static void* client_thread(void* arg){
    int fd=(int)(intptr_t)arg;
    uint8_t sel=0;
    if (recv(fd,&sel,1,MSG_WAITALL)!=1){ close(fd); return NULL; }
    if (sel==0) g_req_video=0;
    else if (sel==2) g_req_video=1;
    uint32_t cap=0; uint8_t* buf=NULL;
    uint64_t last=UINT64_MAX;
    for(;;){
        int is_depth = (sel==1);
        Slot* s = is_depth? &g_depth : &g_video;
        pthread_mutex_t* m = is_depth? &g_dep_mtx : &g_vid_mtx;
        pthread_mutex_lock(m);
        if (s->counter!=last && s->len){
            if (cap < s->len){ free(buf); buf=(uint8_t*)malloc(s->len); cap=s->len; }
            memcpy(buf,s->data,s->len);
            uint32_t w=s->w,h=s->h,fmt=s->fmt,len=s->len; last=s->counter;
            pthread_mutex_unlock(m);
            uint32_t hdr[4];
            hdr[0]=htonl(0x4B494E31u); hdr[1]=htonl(w); hdr[2]=htonl(h); hdr[3]=htonl(fmt);
            if (send_all(fd,hdr,sizeof(hdr))||send_all(fd,buf,len)) break;
        } else {
            pthread_mutex_unlock(m);
            usleep(5000);
        }
    }
    free(buf); close(fd); return NULL;
}

static void* control_thread(void* arg){
    (void)arg;
    int l=socket(AF_INET,SOCK_STREAM,0); int one=1;
    setsockopt(l,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
    struct sockaddr_in a; memset(&a,0,sizeof(a));
    a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_LOOPBACK); a.sin_port=htons(9998);
    if (bind(l,(struct sockaddr*)&a,sizeof(a))||listen(l,4)) return NULL;
    for(;;){
        int c=accept(l,NULL,NULL); if(c<0) continue;
        char line[128]; ssize_t n=recv(c,line,sizeof(line)-1,0);
        char resp[128]="OK\n";
        if (n>0){
            line[n]=0; int iv; float fv;
            if (sscanf(line,"LED %d",&iv)==1 && g_dev) freenect_set_led(g_dev,(freenect_led_options)iv);
            else if (sscanf(line,"TILT %f",&fv)==1 && g_dev) freenect_set_tilt_degs(g_dev,(double)fv);
            else if (!strncmp(line,"ACCEL",5) && g_dev){
                freenect_update_tilt_state(g_dev);
                freenect_raw_tilt_state* st=freenect_get_tilt_state(g_dev);
                double x,y,z; freenect_get_mks_accel(st,&x,&y,&z);
                snprintf(resp,sizeof(resp),"%.3f %.3f %.3f\n",x,y,z);
            }
            else strcpy(resp,"ERR\n");
        }
        send(c,resp,(int)strlen(resp),0);
        close(c);
    }
}

static void* play_thread(void* arg){
    FILE* f=fopen((const char*)arg,"rb");
    if(!f){ fprintf(stderr,"[play] cannot open %s\n",(const char*)arg); return NULL; }
    fprintf(stderr,"[play] looping %s\n",(const char*)arg);
    for(;;){
        uint32_t hdr[6];
        if (fread(hdr,sizeof(hdr),1,f)!=1){ fseek(f,0,SEEK_SET); continue; }
        uint32_t type=ntohl(hdr[0]), dt=ntohl(hdr[1]), magic=ntohl(hdr[2]);
        uint32_t w=ntohl(hdr[3]), h=ntohl(hdr[4]), fmt=ntohl(hdr[5]);
        if (magic!=0x4B494E31u) break;
        uint32_t len = (fmt==0)?w*h*3 : (fmt==1)?w*h*2 : w*h;
        uint8_t* buf=(uint8_t*)malloc(len);
        if (fread(buf,1,len,f)!=len){ free(buf); fseek(f,0,SEEK_SET); continue; }
        if (dt>0 && dt<1000) usleep(dt*1000);
        feed_frame(type==1, w,h,fmt, buf,len);
        free(buf);
    }
    fclose(f); return NULL;
}

static void* freenect_thread(void* arg){
    freenect_context* ctx = (freenect_context*)arg;
    int cur = -1;
    for(;;){
        int want = g_req_video;
        if (want != cur && g_dev){
            freenect_stop_video(g_dev);
            freenect_set_video_mode(g_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, want?FREENECT_VIDEO_IR_8BIT:FREENECT_VIDEO_RGB));
            freenect_start_video(g_dev);
            g_cur_ir = want;
            cur = want;
            fprintf(stderr,"[video] mode -> %s\n", want?"IR8":"RGB");
        }
        struct timeval tv; tv.tv_sec=0; tv.tv_usec=50000;
        if (freenect_process_events_timeout(ctx,&tv) < 0) break;
    }
    return NULL;
}

int main(int argc, char** argv){
    const char* play=NULL; const char* rec=NULL;
    for (int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--play")&&i+1<argc) play=argv[++i];
        else if(!strcmp(argv[i],"--record")&&i+1<argc) rec=argv[++i];
        else if(!strcmp(argv[i],"--ir")) g_use_ir=1;
    }
    if (rec && !play) g_rec=fopen(rec,"wb");

    if (!play){
        if (freenect_init(&g_ctx,NULL)<0){ fprintf(stderr,"freenect_init failed\n"); return 1; }
        if (freenect_open_device(g_ctx,&g_dev,0)<0){ fprintf(stderr,"no kinect\n"); return 1; }
        freenect_set_depth_callback(g_dev,depth_cb);
        freenect_set_video_callback(g_dev,video_cb);
        freenect_set_depth_mode(g_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
        freenect_start_depth(g_dev);
        g_req_video = g_use_ir ? 1 : 0;
        freenect_set_led(g_dev,LED_GREEN);
        pthread_t ft; pthread_create(&ft,NULL,freenect_thread,g_ctx);
        fprintf(stderr,"[live] registered depth(mm) + %s\n", g_use_ir?"IR8":"RGB");
    } else {
        pthread_t pt; pthread_create(&pt,NULL,play_thread,(void*)play);
    }

    pthread_t ct; pthread_create(&ct,NULL,control_thread,NULL);

    int l=socket(AF_INET,SOCK_STREAM,0); int one=1;
    setsockopt(l,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
    struct sockaddr_in a; memset(&a,0,sizeof(a));
    a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_LOOPBACK); a.sin_port=htons(9999);
    if (bind(l,(struct sockaddr*)&a,sizeof(a))||listen(l,8)){ fprintf(stderr,"bind 9999 failed\n"); return 1; }
    fprintf(stderr,"[srv] streaming on 127.0.0.1:9999, control on 9998\n");
    for(;;){
        int c=accept(l,NULL,NULL); if(c<0) continue;
        int one2=1; setsockopt(c,IPPROTO_TCP,TCP_NODELAY,&one2,sizeof(one2));
        pthread_t t; pthread_create(&t,NULL,client_thread,(void*)(intptr_t)c);
        pthread_detach(t);
    }
    return 0;
}

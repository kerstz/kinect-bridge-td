#include <winsock2.h>
#include <ws2tcpip.h>
#include "KinectBridgeTOP.h"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cassert>

static const uint32_t FRAME_MAGIC = 0x4B494E31u;
static const float PC_FX = 594.2143f, PC_FY = 594.2143f, PC_CX = 320.f, PC_CY = 240.f;

static float clamp01(float v){ return v<0?0:(v>1?1:v); }

static void lerpStops(const float s[5][3], float t, float* r, float* g, float* b){
    float x = clamp01(t)*4.f; int i=(int)x; if(i>3)i=3; float f=x-i;
    *r=s[i][0]+(s[i+1][0]-s[i][0])*f;
    *g=s[i][1]+(s[i+1][1]-s[i][1])*f;
    *b=s[i][2]+(s[i+1][2]-s[i][2])*f;
}
static void buildColormap(int which, float* lut){ // 256*3
    static const float VIR[5][3]={{.267f,.005f,.329f},{.283f,.141f,.458f},{.128f,.567f,.551f},{.369f,.789f,.383f},{.993f,.906f,.144f}};
    static const float INF[5][3]={{0,0,.014f},{.287f,.06f,.33f},{.627f,.206f,.227f},{.916f,.486f,.102f},{.988f,.998f,.645f}};
    for(int i=0;i<256;i++){
        float t=i/255.f, r,g,b;
        if(which==1){ r=clamp01(1.5f-fabs(4*t-3)); g=clamp01(1.5f-fabs(4*t-2)); b=clamp01(1.5f-fabs(4*t-1)); }
        else if(which==2) lerpStops(VIR,t,&r,&g,&b);
        else if(which==3) lerpStops(INF,t,&r,&g,&b);
        else r=g=b=t;
        lut[i*3]=r; lut[i*3+1]=g; lut[i*3+2]=b;
    }
}

static uint16_t medianOf(uint16_t* v, int n){
    for(int i=1;i<n;i++){ uint16_t x=v[i]; int j=i-1; while(j>=0&&v[j]>x){v[j+1]=v[j];j--;} v[j+1]=x; }
    return v[n/2];
}
static void denoiseDepth(std::vector<uint16_t>& d, int w, int h){ // médiane 3x3 + bouche-trous
    std::vector<uint16_t> out(d.size()); uint16_t win[9];
    for(int y=0;y<h;y++) for(int x=0;x<w;x++){
        int n=0;
        for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++){
            int yy=y+dy, xx=x+dx;
            if(yy<0||xx<0||yy>=h||xx>=w) continue;
            uint16_t v=d[yy*w+xx]; if(v) win[n++]=v;
        }
        out[y*w+x] = n? medianOf(win,n) : 0;
    }
    d.swap(out);
}

extern "C"
{
    DLLEXPORT void FillTOPPluginInfo(TOP_PluginInfo* info)
    {
        if (!info->setAPIVersion(TOPCPlusPlusAPIVersion)) return;
        info->executeMode = TOP_ExecuteMode::CPUMem;
        info->customOPInfo.opType->setString("Kinectbridge");
        info->customOPInfo.opLabel->setString("Kinect Bridge");
        info->customOPInfo.opIcon->setString("KIN");
        info->customOPInfo.authorName->setString("KinectBridgeTD");
        info->customOPInfo.authorEmail->setString("");
        info->customOPInfo.minInputs = 0;
        info->customOPInfo.maxInputs = 0;
        info->customOPInfo.opHelpURL->setString("");
    }
    DLLEXPORT TOP_CPlusPlusBase* CreateTOPInstance(const OP_NodeInfo* info, TOP_Context* context)
    { return new KinectBridgeTOP(info, context); }
    DLLEXPORT void DestroyTOPInstance(TOP_CPlusPlusBase* instance, TOP_Context* context)
    { (void)context; delete (KinectBridgeTOP*)instance; }
}

KinectBridgeTOP::KinectBridgeTOP(const OP_NodeInfo* info, TOP_Context* context)
    : myNodeInfo(info), myContext(context)
    , mMode(0), mColormap(1), mMirror(false), mFlipV(true), mSwapRB(false), mInvertDepth(false)
    , mAutorange(false), mDenoise(false), mCloseValue(500), mFarValue(4000)
    , mUnknownValue(0.f), mGamma(1.f), mAutoLo(500.f), mAutoHi(4000.f)
    , mWantBg(false), mBgW(0), mBgH(0)
    , mLed(1), mTilt(0.f), mSentLed(-1), mSentTilt(-999.f)
    , mHost("127.0.0.1"), mPort(9999), mDesiredSelector(0)
    , mSocket(INVALID_SOCKET), mRunning(false), mConnected(false)
    , mLatestFormat(-1), mFrameWidth(0), mFrameHeight(0), mFrameCounter(0), mLastConsumedFrame(0)
{
    WSADATA w; WSAStartup(MAKEWORD(2,2), &w);
    startNetworkThread();
}
KinectBridgeTOP::~KinectBridgeTOP(){ stopNetworkThread(); WSACleanup(); }

void KinectBridgeTOP::getGeneralInfo(TOP_GeneralInfo* ginfo, const OP_Inputs*, void*)
{ ginfo->cookEveryFrameIfAsked = true; }

void KinectBridgeTOP::sendControl(const char* line){
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s==INVALID_SOCKET) return;
    DWORD to=100; setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,(const char*)&to,sizeof(to));
    setsockopt(s,SOL_SOCKET,SO_SNDTIMEO,(const char*)&to,sizeof(to));
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(9998);
    inet_pton(AF_INET, mHost.c_str(), &a.sin_addr);
    if (connect(s,(sockaddr*)&a,sizeof(a))==0) send(s,line,(int)strlen(line),0);
    closesocket(s);
}

void KinectBridgeTOP::execute(TOP_Output* output, const OP_Inputs* inputs, void*)
{
    mMode       = inputs->getParInt("Image");
    mColormap   = inputs->getParInt("Colormap");
    mMirror     = inputs->getParInt("Mirror") != 0;
    mFlipV      = inputs->getParInt("Flipvertical") != 0;
    mSwapRB     = inputs->getParInt("Swapredblue") != 0;
    mInvertDepth= inputs->getParInt("Invertdepth") != 0;
    mAutorange  = inputs->getParInt("Autorange") != 0;
    mDenoise    = inputs->getParInt("Denoise") != 0;
    mCloseValue = inputs->getParInt("Close");
    mFarValue   = inputs->getParInt("Far");
    mUnknownValue=(float)inputs->getParDouble("Unknownvalue");
    mGamma      =(float)inputs->getParDouble("Gamma");
    mLed        = inputs->getParInt("Led");
    mTilt       =(float)inputs->getParDouble("Tilt");
    mHost       = inputs->getParString("Host");
    mPort       = inputs->getParInt("Port");

    int sel = (mMode==0)?0 : (mMode==5)?2 : 1;
    mDesiredSelector.store(sel);

    // LED / tilt : envoi uniquement si changement
    if (mLed != mSentLed){ std::string l="LED "+std::to_string(mLed)+"\n"; sendControl(l.c_str()); mSentLed=mLed; }
    if (fabsf(mTilt-mSentTilt) >= 1.f){ char b[32]; snprintf(b,sizeof(b),"TILT %.0f\n",mTilt); sendControl(b); mSentTilt=mTilt; }

    std::vector<uint8_t> payload; int format=-1,w=0,h=0; bool have=false;
    {
        std::lock_guard<std::mutex> lk(mFrameMutex);
        if (mFrameCounter.load()!=mLastConsumedFrame && !mLatestPayload.empty()){
            payload=mLatestPayload; format=mLatestFormat; w=mFrameWidth; h=mFrameHeight;
            mLastConsumedFrame=mFrameCounter.load(); have=true;
        }
    }
    if (!have||w==0||h==0) return;
    if ((mMode==0&&format!=0)||(mMode==5&&format!=2)) return;
    if ((mMode>=1&&mMode<=4)&&format!=1) return;

    std::vector<float> rgba((size_t)w*h*4);

    if (mMode==0 || mMode==5){ // COLOR ou INFRARED (gamma LUT + swap + mirror + flip)
        float lut[256]; float g=mGamma<0.1f?0.1f:mGamma;
        for(int i=0;i<256;i++) lut[i]=powf(i/255.f,1.f/g);
        int ch = (format==0)?3:1;
        for(int y=0;y<h;y++){
            int sy = mFlipV ? (h-1-y) : y;
            const uint8_t* src = payload.data() + (size_t)sy*w*ch;
            float* dst = rgba.data() + (size_t)y*w*4;
            for(int x=0;x<w;x++){
                int sx = mMirror ? (w-1-x) : x;
                const uint8_t* s = src + (size_t)sx*ch;
                if (ch==3){
                    dst[x*4+0]=lut[s[mSwapRB?2:0]];
                    dst[x*4+1]=lut[s[1]];
                    dst[x*4+2]=lut[s[mSwapRB?0:2]];
                } else {
                    dst[x*4+0]=dst[x*4+1]=dst[x*4+2]=lut[s[0]];
                }
                dst[x*4+3]=1.f;
            }
        }
    } else { // modes basés depth (mm)
        std::vector<uint16_t> depth((uint16_t*)payload.data(), (uint16_t*)payload.data()+ (size_t)w*h);
        if (mDenoise) denoiseDepth(depth,w,h);

        if (mWantBg.load()){ mBackground=depth; mBgW=w; mBgH=h; mWantBg=false; }

        float lo=(float)mCloseValue, hi=(float)mFarValue;
        if (mAutorange){
            int hist[128]; memset(hist,0,sizeof(hist)); int total=0;
            for(int i=0;i<w*h;i++){ uint16_t v=depth[i]; if(v<500||v>8000)continue; hist[v/64]++; total++; }
            if (total>500){
                int acc=0; float p2=lo,p98=hi;
                for(int b2=0;b2<128;b2++){ acc+=hist[b2]; if(acc>total*2/100){ p2=(float)(b2*64); break; } }
                acc=0;
                for(int b2=0;b2<128;b2++){ acc+=hist[b2]; if(acc>total*98/100){ p98=(float)((b2+1)*64); break; } }
                if (p98-p2<200) p98=p2+200;
                mAutoLo+=(p2-mAutoLo)*0.15f; mAutoHi+=(p98-mAutoHi)*0.15f;
                lo=mAutoLo; hi=mAutoHi;
            } else { lo=mAutoLo; hi=mAutoHi; }
        }
        if (hi<=lo) hi=lo+1;

        float cmap[256*3];
        if (mMode==1) buildColormap(mColormap,cmap);

        bool bgOk = (mMode==3) && ((int)mBackground.size()==(size_t)w*h) && mBgW==w;

        for(int y=0;y<h;y++){
            int sy = mFlipV ? (h-1-y) : y;
            const uint16_t* src = depth.data() + (size_t)sy*w;
            const uint16_t* bg  = bgOk ? mBackground.data() + (size_t)sy*w : nullptr;
            float* dst = rgba.data() + (size_t)y*w*4;
            for(int x=0;x<w;x++){
                int sx = mMirror ? (w-1-x) : x;
                uint16_t mm = src[sx];
                bool valid = mm>0;
                float f=0;
                if (valid){
                    if (mMode==2) f = clamp01(mm/10000.f);
                    else { f = clamp01(((float)mm-lo)/(hi-lo)); if(mInvertDepth) f=1.f-f; }
                }
                if (mMode==1){ // DEPTH + colormap + alpha validité
                    int ci = (int)(f*255.f);
                    dst[x*4+0]=valid?cmap[ci*3]:mUnknownValue;
                    dst[x*4+1]=valid?cmap[ci*3+1]:mUnknownValue;
                    dst[x*4+2]=valid?cmap[ci*3+2]:mUnknownValue;
                    dst[x*4+3]=valid?1.f:0.f;
                } else if (mMode==2){ // RAW DEPTH gris
                    dst[x*4+0]=dst[x*4+1]=dst[x*4+2]=valid?f:mUnknownValue;
                    dst[x*4+3]=valid?1.f:0.f;
                } else if (mMode==3){ // MASK silhouette
                    bool on = valid && bg && bg[sx]>0 && abs((int)mm-(int)bg[sx])>80 && mm>=lo && mm<=hi;
                    dst[x*4+0]=dst[x*4+1]=dst[x*4+2]=on?1.f:0.f;
                    dst[x*4+3]=on?1.f:0.f;
                } else { // POINTCLOUD XYZ mètres
                    if (valid){
                        float z = mm/1000.f;
                        dst[x*4+0]=((float)x-PC_CX)*z/PC_FX;
                        dst[x*4+1]=((float)y-PC_CY)*z/PC_FY;
                        dst[x*4+2]=z;
                        dst[x*4+3]=1.f;
                    } else { dst[x*4+0]=dst[x*4+1]=dst[x*4+2]=dst[x*4+3]=0.f; }
                }
            }
        }
    }

    TOP_UploadInfo info;
    info.textureDesc.texDim = OP_TexDim::e2D;
    info.textureDesc.width = w;
    info.textureDesc.height = h;
    info.textureDesc.pixelFormat = OP_PixelFormat::RGBA32Float;
    info.colorBufferIndex = 0;
    uint64_t byteSize = uint64_t(w)*h*4*sizeof(float);
    OP_SmartRef<TOP_Buffer> buf = myContext->createOutputBuffer(byteSize, TOP_BufferFlags::None, nullptr);
    assert(buf->size >= byteSize);
    memcpy(buf->data, rgba.data(), byteSize);
    output->uploadBuffer(&buf, info, nullptr);
}

void KinectBridgeTOP::setupParameters(OP_ParameterManager* manager, void*)
{
    { OP_StringParameter p; p.name="Image"; p.label="Image"; p.page="Kinect"; p.defaultValue="color";
      const char* n[]={"color","depth","rawdepth","mask","pointcloud","infrared"};
      const char* l[]={"Color","Depth","Raw Depth","Mask (Silhouette)","Point Cloud XYZ","Infrared"};
      manager->appendMenu(p,6,n,l); }
    { OP_StringParameter p; p.name="Colormap"; p.label="Colormap"; p.page="Kinect"; p.defaultValue="jet";
      const char* n[]={"grey","jet","viridis","inferno"};
      const char* l[]={"Grey","Jet","Viridis","Inferno"};
      manager->appendMenu(p,4,n,l); }
    { OP_NumericParameter p; p.name="Mirror"; p.label="Mirror Image"; p.page="Kinect"; p.defaultValues[0]=0; manager->appendToggle(p); }
    { OP_NumericParameter p; p.name="Flipvertical"; p.label="Flip Vertical"; p.page="Kinect"; p.defaultValues[0]=1; manager->appendToggle(p); }
    { OP_NumericParameter p; p.name="Autorange"; p.label="Auto Range"; p.page="Kinect"; p.defaultValues[0]=0; manager->appendToggle(p); }
    { OP_NumericParameter p; p.name="Denoise"; p.label="Denoise Depth"; p.page="Kinect"; p.defaultValues[0]=0; manager->appendToggle(p); }
    { OP_NumericParameter p; p.name="Swapredblue"; p.label="Swap Red/Blue"; p.page="Kinect"; p.defaultValues[0]=0; manager->appendToggle(p); }
    { OP_NumericParameter p; p.name="Invertdepth"; p.label="Invert Depth"; p.page="Kinect"; p.defaultValues[0]=0; manager->appendToggle(p); }
    { OP_NumericParameter p; p.name="Unknownvalue"; p.label="Unknown Value"; p.page="Kinect"; p.defaultValues[0]=0; p.minSliders[0]=0; p.maxSliders[0]=1; manager->appendFloat(p); }
    { OP_NumericParameter p; p.name="Gamma"; p.label="Gamma"; p.page="Kinect"; p.defaultValues[0]=1; p.minSliders[0]=0.2f; p.maxSliders[0]=3; manager->appendFloat(p); }
    { OP_NumericParameter p; p.name="Close"; p.label="Too Close Value (mm)"; p.page="Kinect"; p.defaultValues[0]=500; p.minSliders[0]=0; p.maxSliders[0]=8000; manager->appendInt(p); }
    { OP_NumericParameter p; p.name="Far"; p.label="Too Far Value (mm)"; p.page="Kinect"; p.defaultValues[0]=4000; p.minSliders[0]=0; p.maxSliders[0]=10000; manager->appendInt(p); }
    { OP_NumericParameter p; p.name="Capturebg"; p.label="Capture Background"; p.page="Kinect"; manager->appendPulse(p); }
    { OP_StringParameter p; p.name="Led"; p.label="LED"; p.page="Device"; p.defaultValue="green";
      const char* n[]={"0","1","2","3","4","5","6"};
      const char* l[]={"Off","Green","Red","Yellow","Blink Yellow","Blink Green","Blink Red"};
      manager->appendMenu(p,7,n,l); }
    { OP_NumericParameter p; p.name="Tilt"; p.label="Tilt (deg)"; p.page="Device"; p.defaultValues[0]=0; p.minSliders[0]=-30; p.maxSliders[0]=30; manager->appendFloat(p); }
    { OP_StringParameter p; p.name="Host"; p.label="Bridge Host"; p.page="Network"; p.defaultValue="127.0.0.1"; manager->appendString(p); }
    { OP_NumericParameter p; p.name="Port"; p.label="Bridge Port"; p.page="Network"; p.defaultValues[0]=9999; p.minSliders[0]=1024; p.maxSliders[0]=65535; manager->appendInt(p); }
    { OP_NumericParameter p; p.name="Reconnect"; p.label="Reconnect"; p.page="Network"; manager->appendPulse(p); }
}

void KinectBridgeTOP::pulsePressed(const char* name, void*)
{
    if (strcmp(name,"Reconnect")==0){ closesocket(mSocket); mSocket=INVALID_SOCKET; mConnected=false; }
    if (strcmp(name,"Capturebg")==0){ mWantBg=true; }
}

void KinectBridgeTOP::startNetworkThread(){ mRunning=true; mNetThread=std::thread(&KinectBridgeTOP::networkLoop,this); }
void KinectBridgeTOP::stopNetworkThread(){
    mRunning=false;
    if (mSocket!=INVALID_SOCKET) closesocket(mSocket);
    if (mNetThread.joinable()) mNetThread.join();
}
bool KinectBridgeTOP::connectSocket(){
    mSocket=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if (mSocket==INVALID_SOCKET) return false;
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons((u_short)mPort);
    inet_pton(AF_INET,mHost.c_str(),&a.sin_addr);
    if (connect(mSocket,(sockaddr*)&a,sizeof(a))!=0){ closesocket(mSocket); mSocket=INVALID_SOCKET; return false; }
    char s=(char)mDesiredSelector.load();
    send(mSocket,&s,1,0);
    return true;
}
bool KinectBridgeTOP::recvExact(char* buf, int len){
    int got=0;
    while(got<len){ int n=recv(mSocket,buf+got,len-got,0); if(n<=0)return false; got+=n; }
    return true;
}
void KinectBridgeTOP::networkLoop(){
    int curSel=-1;
    while(mRunning){
        int want=mDesiredSelector.load();
        if (mSocket!=INVALID_SOCKET && want!=curSel){ closesocket(mSocket); mSocket=INVALID_SOCKET; mConnected=false; }
        if (mSocket==INVALID_SOCKET){
            if (!connectSocket()){ std::this_thread::sleep_for(std::chrono::milliseconds(500)); continue; }
            curSel=mDesiredSelector.load(); mConnected=true;
        }
        uint32_t header[4];
        if (!recvExact((char*)header,16)){ closesocket(mSocket); mSocket=INVALID_SOCKET; mConnected=false; continue; }
        uint32_t magic=ntohl(header[0]), w=ntohl(header[1]), h=ntohl(header[2]), fmt=ntohl(header[3]);
        uint32_t size = (fmt==0)?w*h*3 : (fmt==1)?w*h*2 : (fmt==2)?w*h : 0;
        if (magic!=FRAME_MAGIC || size==0 || size>64u*1024u*1024u){ closesocket(mSocket); mSocket=INVALID_SOCKET; mConnected=false; continue; }
        std::vector<uint8_t> payload(size);
        if (!recvExact((char*)payload.data(),(int)size)){ closesocket(mSocket); mSocket=INVALID_SOCKET; mConnected=false; continue; }
        std::lock_guard<std::mutex> lk(mFrameMutex);
        mLatestPayload=std::move(payload);
        mLatestFormat=(int)fmt; mFrameWidth=(int)w; mFrameHeight=(int)h;
        mFrameCounter++;
    }
    if (mSocket!=INVALID_SOCKET) closesocket(mSocket);
}

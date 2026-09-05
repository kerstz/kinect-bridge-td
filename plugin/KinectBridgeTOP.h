#pragma once
#include "TOP_CPlusPlusBase.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <cstdint>
#include <string>

#pragma comment(lib, "ws2_32.lib")
using namespace TD;

class KinectBridgeTOP : public TOP_CPlusPlusBase
{
public:
    KinectBridgeTOP(const OP_NodeInfo* info, TOP_Context* context);
    virtual ~KinectBridgeTOP();
    virtual void getGeneralInfo(TOP_GeneralInfo*, const OP_Inputs*, void*) override;
    virtual void execute(TOP_Output* output, const OP_Inputs* inputs, void*) override;
    virtual void setupParameters(OP_ParameterManager* manager, void*) override;
    virtual void pulsePressed(const char* name, void*) override;

private:
    void startNetworkThread();
    void stopNetworkThread();
    void networkLoop();
    bool connectSocket();
    bool recvExact(char* buf, int len);
    void sendControl(const char* line);

    const OP_NodeInfo* myNodeInfo;
    TOP_Context* myContext;

    int  mMode;          // 0 color 1 depth 2 rawdepth 3 mask 4 pointcloud 5 infrared
    int  mColormap;      // 0 grey 1 jet 2 viridis 3 inferno
    bool mMirror, mFlipV, mSwapRB, mInvertDepth, mAutorange, mDenoise;
    int  mCloseValue, mFarValue;
    float mUnknownValue, mGamma;
    float mAutoLo, mAutoHi;
    std::atomic<bool> mWantBg;
    std::vector<uint16_t> mBackground; int mBgW, mBgH;
    int mLed; float mTilt; int mSentLed; float mSentTilt;

    std::string mHost; int mPort;
    std::atomic<int> mDesiredSelector;

    SOCKET mSocket;
    std::thread mNetThread;
    std::atomic<bool> mRunning, mConnected;

    std::mutex mFrameMutex;
    std::vector<uint8_t> mLatestPayload;
    int mLatestFormat; int mFrameWidth, mFrameHeight;
    std::atomic<uint64_t> mFrameCounter;
    uint64_t mLastConsumedFrame;
};

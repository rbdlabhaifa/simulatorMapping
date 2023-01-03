#include "RaspberryKeyFrame.h"
// #include "KeyFrame.h"
// #include "ORBmatcher.h"
// #include<mutex>

namespace ORB_SLAM2
{

    RaspberryKeyFrame::RaspberryKeyFrame(KeyFrame &kf) : image(kf.image), fx(kf.fx), fy(kf.fy), cx(kf.cx), cy(kf.cy),
                                                         invfx(kf.invfx), invfy(kf.invfy),
                                                         mvKeys(kf.mvKeys), mvKeysUn(kf.mvKeysUn),
                                                         mDescriptors(kf.mDescriptors.clone()), Tcw(kf.GetPose()),
                                                         mvpMapPoints(kf.GetMapPointMatches())
    {
    }

}

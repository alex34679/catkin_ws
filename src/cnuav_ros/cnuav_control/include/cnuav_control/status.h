#ifndef SRC_STATUS_H
#define SRC_STATUS_H

namespace cnuav {

    enum Status{
        Disarm, //加锁
        TakeOff,
        Hover,
        Trajectory,
        Circle,
        Land,
        Slowdown,
    };

}


#endif //SRC_STATUS_H

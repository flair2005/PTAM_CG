#pragma once
#ifndef __Common_H
#define __Common_H

namespace GS
{
struct Size
{
    unsigned int width;
    unsigned int height;
    Size(){};
    Size(unsigned int w,unsigned int h):width(w),height(h){}
};

}
#endif

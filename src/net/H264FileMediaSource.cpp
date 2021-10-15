#include <cstdio>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "net/H264FileMediaSource.h"
#include "base/Logging.h"
#include "base/New.h"

static inline int startCode3(uint8_t* buf);
static inline int startCode4(uint8_t* buf);

H264FileMediaSource* H264FileMediaSource::createNew(UsageEnvironment* env, std::string file)
{
    //return new H264FileMediaSource(env, file);
    return New<H264FileMediaSource>::allocate(env, file);
}

H264FileMediaSource::H264FileMediaSource(UsageEnvironment* env, const std::string& file) :
    MediaSource(env),
    mFile(file)
{
    if(0 == strcmp("-" ,file.c_str()) )
    {
      mFd = STDIN_FILENO;
    }
    else
    {
      mFd = ::open(file.c_str(), O_RDONLY);
    }
    assert(mFd > -1);

    setFps(24);

    for(int i = 0; i < DEFAULT_FRAME_NUM; ++i)
        mEnv->threadPool()->addTask(mTask);
}

H264FileMediaSource::~H264FileMediaSource()
{
    ::close(mFd);
}



static int copy_nal_from_file(int fd, uint8_t *buf, int *len)
{
    char tmpbuf[4];     /* i have forgotten what this var mean */
    char tmpbuf2[1];    /* i have forgotten what this var mean */
    int flag = 0;       /* i have forgotten what this var mean */
    int ret;

#if 0
    ret = read(fd,tmpbuf, 4);
    if (!ret)
        return 0;
#endif

    *len = 0;

    do {
        ret = read(fd,tmpbuf2, 1);
        if (!ret) {
            return -1;
        }
        if (!flag && tmpbuf2[0] != 0x0) {
            buf[*len] = tmpbuf2[0];
            (*len)++;
            // debug_print("len is %d", *len);
        } else if (!flag && tmpbuf2[0] == 0x0) {
            flag = 1;
            tmpbuf[0] = tmpbuf2[0];
            //debug_print("len is %d", *len);
        } else if (flag) {
            switch (flag) {
            case 1:
                if (tmpbuf2[0] == 0x0) {
                    flag++;
                    tmpbuf[1] = tmpbuf2[0];
                } else {
                    flag = 0;
                    buf[*len] = tmpbuf[0];
                    (*len)++;
                    buf[*len] = tmpbuf2[0];
                    (*len)++;
                }
                break;
            case 2:
                if (tmpbuf2[0] == 0x0) {
                    flag++;
                    tmpbuf[2] = tmpbuf2[0];
                } 
                /*
                else if (tmpbuf2[0] == 0x1) {
                    flag = 0;
                    return *len;
                }
                */
                else {
                    flag = 0;
                    buf[*len] = tmpbuf[0];
                    (*len)++;
                    buf[*len] = tmpbuf[1];
                    (*len)++;
                    buf[*len] = tmpbuf2[0];
                    (*len)++;
                }
                break;
            case 3:
                if (tmpbuf2[0] == 0x1) {
                    flag = 0;
                    return *len;
                } else {
                    flag = 0;
                    break;
                }
            }
        }

    } while (1);

    return *len;
}

void H264FileMediaSource::readFrame()
{
    MutexLockGuard mutexLockGuard(mMutex);

    if(mAVFrameInputQueue.empty())
        return;

    AVFrame* frame = mAVFrameInputQueue.front();

    do
    {
      frame->mFrameSize = getFrameFromH264File(mFd, frame->mBuffer, FRAME_MAX_SIZE);

    }while(frame->mFrameSize <= 4);

    frame->mFrame = frame->mBuffer;

    printf("frame->mFrameSize =0x%x\n",frame->mFrameSize );

    static bool bWrite = 0;
    if (!bWrite)
    {
      bWrite = 1;
      int fd1 = open("./frame.data", O_CREAT | O_WRONLY);
      write(fd1,frame->mFrame,frame->mFrameSize);
      close(fd1);
    }

    mAVFrameInputQueue.pop();
    mAVFrameOutputQueue.push(frame);
}

static inline int startCode3(uint8_t* buf)
{
    if(buf[0] == 0 && buf[1] == 0 && buf[2] == 1)
        return 1;
    else
        return 0;
}

static inline int startCode4(uint8_t* buf)
{
    if(buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 1)
        return 1;
    else
        return 0;
}

static uint8_t* findNextStartCode(uint8_t* buf, int len)
{
    int i;

    if(len < 3)
        return NULL;

    for(i = 0; i < len-3; ++i)
    {
        if(startCode3(buf) || startCode4(buf))
            return buf;
        
        ++buf;
    }

    if(startCode3(buf))
        return buf;

    return NULL;
}

int H264FileMediaSource::getFrameFromH264File(int fd, uint8_t* frame, int size)
{

    if(fd < 0)
        return fd;
    int len = size - 4 ;
    
    char nal[4];
    nal[0] = 0;
    nal[1] = 0;
    nal[2] = 0;
    nal[3] = 1;
    memcpy(frame,nal,4);

    len =  4 + copy_nal_from_file(fd,frame+4,&len);
    
    
    return len;
}

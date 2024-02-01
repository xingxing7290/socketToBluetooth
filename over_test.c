#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <bluetooth.h>
#include <sys/ioctl.h>
#include <bluetooth.h>
#include <hci.h>
#include <hci_lib.h>
#include <rfcomm.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/time.h>

typedef unsigned char BYTE;
typedef unsigned long long u64;
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;
typedef signed long long s64;
typedef signed int s32;
typedef signed short s16;
typedef signed char s8;

typedef struct
{
    u16 flag;    // 0xFCCA
    u16 version; // 101
    u32 size;    // 帧长度
    u8 format;   // 数据格式: 1-binary,2-json
    u8 encrypt;  // 加密: 0-无, 1-RSA 2-AES
    u8 compress; // 压缩: 0-无, 1-snappy
    u8 dtype;    // 二进制数据类型(1-图像数据 2-RPC)
    u8 rev[4];
} sk_head;
int usb;
sk_head s;
sk_head s2;

static struct timeval start_tv;
static struct timeval stop_tv;

int svr_start();
static void svr_listen(void);
static void svr_process(void *fd);
int relay_start();
static void relay_listen(void);
static void relay_process(void *fd);
void usb_listen(void);

int heart_start();
static void heart_listen(void);
static void heart_process(void *fd);
void BTconn_listen(void);
void OpenBlueTooth();

void *BTrecv_func(void *p);
void send_conn_status(int connflag);
void *Monitor4g();

static unsigned int GetTimeInterval(struct timeval src_tv, struct timeval dest_tv);

int errorflag = 0;   // 发送给USB 失败的话，就说明是断开了
int errorconsvr = 0; // 16001 断开连接//初始状态是关闭16001 的状态
                     // 但初始值为1 是，就会出现open之后咱直接断开
int errorconrel = 0; // 17001断开连接

int BTerrorconsvr = 0; // 16001 断开连接//初始状态是关闭16001 的状态
                       // 但初始值为1 是，就会出现open之后咱直接断开
int BTerrorconrel = 0; // 17001断开连接

int connflag = 0; // 心跳标志

int abc = 1;  // usb 0-断开 1-连接
int babc = 1; // bluetooth

int USBorBTflag = 0; // 连接方式标志位 0-USB 1-BT
/*******************************************/
int fd_svr = 0;
int fd_relay = 0;
int fd_heart = -1;
int ss = 19000; // 蓝牙socket等待连接；
/*******************************************/
int tcp_server = 16000;   // 16001中级服务器端口
int relay_server = 17000; // 17001 业务服务器端口
int heart_server = 18000; // 18001 心跳包端口
int bt_fd = -1;           // 蓝牙连接描述符
/*******************************************/

/****************蓝牙连接变量***************************/
#define ClientMax 1
#define BUFSIZE 512

char recBuf[BUFSIZE] = {0}; // 用于记录接入的客户端的mac地址
int recvheartflag = 1;      // 蓝牙连接断开标志位，初始值位断开 0：连接 1：断开
/*******************************************/

int usbreconnect = 0; // 用于多指令重连标志

struct timespec last_message_time, current_time;

pthread_mutex_t errormutex;
int heart_flag = 0;

time_t heart_time;
int NotRecHeartNum = 0; // 没有接收到心跳包的次数；

void USBconn_listen(void)
{
    while (1)
    {
        if (connflag == 1) // 接收到了心跳包
        {
            pthread_mutex_lock(&errormutex);
            connflag = 0;
            errorconsvr = 0;
            errorconrel = 0;
            errorflag = 0;
            abc = 0;
            USBorBTflag = 0; // 设置连接模式标志位为USB
            send_conn_status(USBorBTflag);
            time_t now = time(NULL);
            pthread_mutex_unlock(&errormutex);
            NotRecHeartNum = 0;
        }
        else // 没收到心跳包，
        {
            // 如果现在的时间减去上一个心跳的时间大于三秒就说明断开了
            time_t currentTime = time(NULL);
            if (currentTime - heart_time >= 3.2)
            {
                // 断开连接
                pthread_mutex_lock(&errormutex);
                // 打印时间
                time_t now = time(NULL);
                struct tm *t = localtime(&now);
                // printf("当前时间: %d-%02d-%02d %02d:%02d:%02d ",t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,t->tm_hour, t->tm_min, t->tm_sec);
                printf("               USB %d没收到心跳包\n", usb);

                if (USBorBTflag == 0)
                {
                    abc = 1;
                    errorconsvr = 1; // 让连接的16001断开标志位
                    errorconrel = 1; // 直接设置标志位，让17001连接断开;
                }
                pthread_mutex_unlock(&errormutex);

                close(usb);
                // 重新打开USB设备
                usb = open("/dev/ttyGS0", O_RDWR | O_NOCTTY); //| O_NDELAY); // 断开之后重新打开
                struct termios options2;
                tcgetattr(usb, &options2);
                options2.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
                options2.c_iflag = IGNPAR;
                options2.c_oflag = 0;
                options2.c_lflag = 0;
                tcflush(usb, TCIFLUSH); // 清空端口缓冲
                tcsetattr(usb, TCSANOW, &options2);
                // printf("重新打开USB:%d \n", usb);
                heart_time = currentTime; // 重新打开之后重置三秒的初始时间

                NotRecHeartNum++;
            }
        }
        usleep(500 * 1000);
    }
}
int blueoutline = 0; // 0-插上 大于1 拔掉  蓝牙模块拔掉了

void Monitor_NorRecvHeart(void) // 监听没有收到心跳包
{
    while (1)
    {
        if (NotRecHeartNum >= 2 && USBorBTflag == 0) // 两次没有收到心跳包//之后第二次才会进来，第三次的话就不打开了
        {
            abc = 1;         // USB 断开了
            USBorBTflag = 1; // 设置连接标志位为蓝牙；
            OpenBlueTooth(); // 打开蓝牙
        }
        usleep(100 * 1000);
        if (blueoutline == 1) // 蓝牙拔掉了就重新连接一下蓝牙
        {
            OpenBlueTooth(); // 打开蓝牙
        }
        usleep(200 * 1000);
    }
}

void BTconn_listen(void) // 去监听蓝牙连接
{
    while (1)
    {
        if (recvheartflag == 0) // 连接上
        {
            pthread_mutex_lock(&errormutex);
            errorconsvr = 0; // 16001 的连接标志
            errorconrel = 0; // 17001 的连接标志
            babc = 0;
            pthread_mutex_unlock(&errormutex);
        }
        if (recvheartflag == 1) // 断开
        {
            pthread_mutex_lock(&errormutex);
            babc = 1;
            errorconsvr = 1;
            errorconrel = 1; // 直接设置标志位，让所有的连接断开;
            pthread_mutex_unlock(&errormutex);
            printf("BTconn_listen close\n");
            return; // 断开之后跳出循环结束这个线程，然后上层线程就结束
        }
        usleep(500 * 1000); // 休眠半秒
                            // printf("BTconn_listen\n");
    }
}

int usb;
char end = '\0';
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; // 初始化互斥锁

void sigint_handler(int sig)
{
    if (tcp_server != 0)
        close(tcp_server); // 关闭套接字
    if (tcp_server != 0)
        close(relay_server);
    if (heart_server != 0)
        close(heart_server);
    if (ss != 0)
        close(ss);

    exit(0); // 退出程序
}

int main()
{
    signal(SIGINT, sigint_handler); // 处理ctrl+c中断

    // 执行命令并获取返回值
    int returnValue = system("sdptool add SP");

    // 检查命令执行是否成功
    if (returnValue == -1)
        printf("命令执行失败\n");
    else
        printf("命令执行成功\n");
    pthread_mutex_init(&errormutex, NULL);
    while (1)
    {
        // 打开串口
        do
        {
            usb = open("/dev/ttyGS0", O_RDWR | O_NOCTTY); //| O_NDELAY);设置为阻塞模式
            if (usb < 0)
            {
                sleep(1);
            }
            else
            {
                printf("串口已打开\n");
                break;
            }
            usleep(1000);
        } while (1);

        struct termios options;
        tcgetattr(usb, &options);
        options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcflush(usb, TCIFLUSH);
        tcsetattr(usb, TCSANOW, &options);

        pthread_t usbid; // USB接收线程
        pthread_create(&usbid, NULL, (void *)usb_listen, NULL);
        printf("USB接收线程已启动\n");
        /***************************************************************************/
        pthread_attr_t connid_attr; // 创建线程属性
        pthread_attr_init(&connid_attr);
        pthread_attr_setstacksize(&connid_attr, 1024 * 1024); // 设置栈大小 <1MB
        pthread_t connid;                                     // USb 监听心跳包线程
        pthread_create(&connid, &connid_attr, (void *)USBconn_listen, NULL);
        printf("USb 监听心跳包线程已启动\n");
        /***************************************************************************/
        pthread_attr_t svr_startid_attr; // 创建线程属性
        pthread_attr_init(&svr_startid_attr);
        pthread_attr_setstacksize(&svr_startid_attr, 2 * 1024 * 1024);
        pthread_t svr_startid; // 启动16001
        pthread_create(&svr_startid, &svr_startid_attr, (void *)svr_start, NULL);
        printf("启动16001线程已启动\n");
        /***************************************************************************/
        pthread_attr_t Monitor_NorRecvHeart_attr; // 创建线程属性
        pthread_attr_init(&Monitor_NorRecvHeart_attr);
        pthread_attr_setstacksize(&Monitor_NorRecvHeart_attr, 1024 * 1024); // 设置栈大小 <1MB
        pthread_t Monitor_NorRecvHeartid;                                   // 启动丢失监听心跳包线程
        pthread_create(&Monitor_NorRecvHeartid, &Monitor_NorRecvHeart_attr, (void *)Monitor_NorRecvHeart, NULL);
        printf("启动丢失监听心跳包线程已启动\n");
        /***************************************************************************/
        pthread_attr_t Monitor4g_attr; // 创建线程属性
        pthread_attr_init(&Monitor4g_attr);
        pthread_attr_setstacksize(&Monitor4g_attr, 1024 * 1024); // 设置栈大小 <1MB
        pthread_t Monitor4g_id;
        pthread_create(&Monitor4g_id, &Monitor4g_attr, (void *)Monitor4g, NULL);
        printf("启动4G模块监听线程已启动\n");
        /***************************************************************************/

        relay_start(); // 启动17001
        heart_start(); // 启动18001

        while (1)
        {
            usleep(60000 * 1000);
        }
    }
}

/*检测心跳包还有蓝牙模块
 */
void *Monitor4g()
{
    while (1)
    {
        usleep(3000 * 1000);                         // 没三秒检测一次
        if (NotRecHeartNum >= 2 && blueoutline >= 2) // 既没有心跳包，有没有蓝牙
        {
            send_conn_status(2);
            while (1) // 当检测到心跳包的时候，或者蓝牙插上的时候退出//当没有心跳包，或者蓝牙拔掉的时候，就在这循环等待
            {
                if (NotRecHeartNum == 0)
                    break;
                if (blueoutline == 0)
                    break;
                usleep(3000 * 1000);
            }
        }
    }
}
// 在蓝牙状态下，既要accept去等待连接，又要去判断这个NotRecHeartNum心跳包数量是否为0
// 但是accept使用阻塞模式时，会一直阻塞原地，而无法进行后面的判断
// 但是accept使用非阻塞模式时，他判断的很快，还没等连接上呢，就又重新打开了这个accept了；
void OpenBlueTooth()
{
    printf("Enter OpenBlueTooth\n");

    if (system("hciconfig hci0 piscan") != 0)
    {
        blueoutline++; // 让本机蓝牙处于可见状态

        while (system("hciconfig hci0 piscan") != 0)
        {
            printf("Waiting for Bluetooth device to be inserted... %d\n", blueoutline);
            blueoutline++;
            sleep(10);
            if (NotRecHeartNum == 0) // USB 连接上了
                return;
        }
    }

    pthread_mutex_init(&errormutex, NULL);
    int c_fd[ClientMax];
    struct sockaddr_rc loc_addr = {0}, rem_addr = {0};
    int bytes_read, i, err, ret;
    pthread_t rec_tid[ClientMax] = {0};
    pthread_t send_tid;
    int opt = sizeof(rem_addr);

    blueoutline = 0; // 拔掉蓝牙之后重新插上
    send_conn_status(USBorBTflag);

    ss = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (ss == -1)
    {
        perror("Error opening Bluetooth socket");
        return;
    }
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = *BDADDR_ANY; // 相当于tcp的ip地址
    loc_addr.rc_channel = (uint8_t)1; // 这里的通道就是SPP的通道，相当于网络编程里的端口

    ret = bind(ss, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
    if (ret < 0)
    {
        perror("Error binding socket");
        return;
    }
    // 监听客户端连接
    ret = listen(ss, ClientMax);
    if (ret < 0)
    {
        perror("Error listening on Bluetooth socket");
        return;
    }
    printf("bluetooth_server listen success  ss:%d\n", ss);
    // 设置套接字为非阻塞模式
    int flags = fcntl(ss, F_GETFL, 0);
    fcntl(ss, F_SETFL, flags | O_NONBLOCK);
    // 初始化客户端套接字
    for (i = 0; i < ClientMax; i++)
    {
        c_fd[i] = -1;
    }

    // 不断等待是否有新蓝牙接入
    while (NotRecHeartNum != 0)
    {
        i = 0;
        if (recvheartflag == 1) // 如果是蓝牙断开状态的话，才去连接
        {
            printf("Accepting  ss &%d\n", ss);
            c_fd[i] = accept(ss, (struct sockaddr *)&rem_addr, &opt);
            if (c_fd[i] > 0) // bt_fd
            {
                bt_fd = c_fd[i];
                printf("client connected success      %d\n", c_fd[i]);
                recvheartflag = 0; // 连接上的标志
                ba2str(&rem_addr.rc_bdaddr, recBuf);
                fprintf(stdout, "accepted connection from %s\n", recBuf);
                // 为每个新的客户端创建自己的线程用于接收信息
                err = pthread_create((rec_tid + i), NULL, BTrecv_func, (c_fd + i));
                printf("创建蓝牙接收线程 \n");

                if (err)
                {
                    fprintf(stderr, "Create pthread fail: %s\n", strerror(err));
                    exit(1);
                }
                pthread_join(rec_tid[i], NULL); // 等待接收消息线程退出结束。
                close(c_fd[i]);

                // 如果检测到蓝牙模块拔掉  就break退出这个循环，进行下一次的打开蓝牙
                printf("bluetooth recv closed2\n");
            }
            if (system("hciconfig hci0 up") != 0) // 设备没有插在上面
            {                                     // 在这里可以while循环等，当system返回0 或者当NotRecHeartNum==0 连接USB 时断开
                printf("Waiting for Bluetooth device to be inserted...\n");
                blueoutline = 1;
                break;
            }
            usleep(3 * 1000 * 1000);
        }
        usleep(500 * 1000);
    }
    if (NotRecHeartNum == 0) // USB 收到心跳包了，就直接退出 结束这个进程
    {

        if (c_fd[i] != 0)
        {
            printf("close c_fd[i] %d \n", c_fd[i]);
            close(c_fd[i]);
        }
        if (ss != 0)
        {
            printf("close ss %d \n", ss);
            close(ss);
        }
        printf("收到心跳包断开 BT Connection Closed\n");
        return;
    }
    if (c_fd[i] != 0)
        close(c_fd[i]);
    if (ss != 0)
        close(ss);
    printf(" 蓝牙断开 00 BT Connection Closed\n");
    return;
}

void usb_listen(void) // 这个线程要一直去监听，不能结束跳出；
{
    // 初始化时间戳
    clock_gettime(CLOCK_MONOTONIC, &last_message_time);
    int USBlen = -1;
    char buff[16432 * 10]; // 2049
    char buff2[16432 * 10];
    while (1)
    {
        // 接收USB消息
        USBlen = read(usb, buff, sizeof(buff)); // 阻塞进程

        if (USBlen > 0)
        {
            uintptr_t ptr2 = (uintptr_t)buff;
            sk_head *head2 = (sk_head *)ptr2;
            int size = head2->size + 16 + 1;
            if (size < 0 || size > 16342 * 10) // 处理炻器物联上传文件用的
            {
                /**转发的时候，炻器物联发送时间要有间隔，不然的话，会出现接收但是转发不成功的问题
                 *就和设备给炻器物联发送文件的情况一样了
                 */
                // int states = 0;
                // printf("usblen %d <=> size :%d \n", USBlen, size);
                // if (buff[USBlen - 1] == '7')
                // {
                //     buff[USBlen - 1] = '\0';
                //     states = send(fd_relay, buff, USBlen - 1, 0);
                //     printf("包结尾\n");
                // }
                // else
                // {
                //     states = send(fd_relay, buff, USBlen, 0);
                // }
                // if (states >= 0)
                // {
                //     printf("上传send %d to socket 17001 socket %d\n", USBlen, states);
                //     // memset(buff, 0, sizeof(buff));
                // }
                // else
                // {
                //     printf("Send failed with error: %s\n", strerror(errno));
                // }
            }
            else
            {
                if (USBlen > size)
                {
                    printf("usblen %d >> size :%d \n", USBlen, size);
                    int offset = 0;
                    while (offset < USBlen)
                    {
                        sk_head *current_head = (sk_head *)(buff + offset); // 获取当前指令的头部
                        int current_size = current_head->size + 16 + 1;     // 计算当前指令的总长度
                        char buffer[current_size];
                        memcpy(buffer, buff + offset, current_size); // 将当前指令+标志位+复制到新的缓冲区
                        int states;
                        if (buffer[current_size - 1] == '6')
                        {
                            states = send(fd_svr, buffer, current_size - 1, 0);
                        }
                        else if (buffer[current_size - 1] == '7')
                        {
                            states = send(fd_relay, buffer, current_size - 1, 0);
                        }
                        else if (buffer[current_size - 1] == '9')
                        {
                            connflag = 1;
                            printf("receive heartbeat\n");
                        }
                        else // 出现了粘包现象。两个指令连在一起只有最后一位标志位
                        {
                            current_size = current_size - 1;
                            states = send(fd_relay, buffer, current_size, 0);
                        }

                        if (states >= 0)
                        {
                            printf("send %d to socket 17001 socket %d\n", USBlen, states);
                        }
                        else if (states == -1)
                        {
                            printf("Send failed with error: %s\n", strerror(errno));
                        }
                        offset += current_size; // 更新偏移量以处理下一条指令
                    }
                    memset(buff, 0, sizeof(buff)); // 在处理完所有指令后清空buf
                }
                else if (USBlen == size)
                {
                    printf("usblen %d == size :%d\n", USBlen, size);
                    //  发送TCP消息
                    if (buff[USBlen - 1] == '6')
                    {
                        buff[USBlen - 1] = end;
                        uintptr_t ptr = (uintptr_t)buff;
                        sk_head *head = (sk_head *)ptr;
                        memcpy(buff2, &buff[sizeof(s)], USBlen - sizeof(s));
                        printf("read data from 16001 USB:%s\n", buff2);

                        int states = send(fd_svr, buff, USBlen - 1, 0);
                        if (states >= 0)
                        {
                            printf("send  %s  to socket 16001\n", buff2);
                            memset(buff, 0, sizeof(buff));
                        }
                        else if (states == -1)
                        {
                            printf("Send failed with error: %s\n", strerror(errno));
                        }
                    }
                    else if (buff[USBlen - 1] == '7')
                    {
                        buff[USBlen - 1] = end;
                        uintptr_t ptr = (uintptr_t)buff;
                        sk_head *head = (sk_head *)ptr;
                        memcpy(buff2, &buff[sizeof(s)], sizeof(buff) - sizeof(s));
                        //  发送TCP消息
                        printf("read %d data from 17001 USB:%s\n", USBlen, buff2);
                        int states = send(fd_relay, buff, USBlen - 1, 0);
                        if (states >= 0)
                        {
                            printf("send %d to socket 17001 socket %d\n", USBlen, states);
                            memset(buff, 0, sizeof(buff));
                        }
                        else if (states == -1)
                        {
                            printf("Send failed with error: %s\n", strerror(errno));
                        }
                    }
                    else if (buff[USBlen - 1] == '8') // 新指令
                    {
                        usbreconnect = 1;
                        buff[USBlen - 1] = end;
                        uintptr_t ptr = (uintptr_t)buff;
                        sk_head *head = (sk_head *)ptr;
                        memcpy(buff2, &buff[sizeof(s)], USBlen - sizeof(s));
                        printf("read data from 16001 USB:%s\n", buff2);

                        int states = send(fd_svr, buff, USBlen - 1, 0);
                        if (states >= 0)
                        {
                            printf("send %s  to socket 16001\n", buff2);
                            memset(buff, 0, sizeof(buff));
                        }
                        else if (states == -1)
                        {
                            printf("Send failed with error: %s\n", strerror(errno));
                        }
                    }
                    else if (buff[USBlen - 1] == '9') // 连接心跳
                    {
                        connflag = 1;
                        time_t now = time(NULL);

                        // 将时间戳转换为本地时间
                        struct tm *t = localtime(&now);

                        // 打印时间
                        printf("USB receive heartbeat\n");
                        // 获取时间
                        heart_time = time(NULL);
                    }
                    else
                    {
                        int states = send(fd_svr, buff, USBlen, 0);
                        if (states >= 0)
                        {
                            printf("2 send %s  to socket 16001\n", buff2);
                            memset(buff, 0, sizeof(buff));
                        }
                    }
                }
            }
        }
        else
        {
            printf("USB recv fails \n");
            // 客户端断开连接，则关闭对应的socket
            // printf("USB %d中断或者下线了\n", usb);
            usleep(100 * 1000);
        }
    }
    printf("usb listen pthread closed\n");
}

void *BTrecv_func(void *p)
{

    int tmp_c_fd = *((int *)p); // 拿到接入的客户端的套接字

    char nameBuf[BUFSIZE] = {0}; // 存储接入的客户端的mac地址,用于区别不同客户端
    char readBuf[16432] = {0};   // 用于存储接收到对应客户端的消息
    int n_read = 0;

    // 将全局变量recBuf接收到的mac地址，copy到nameBuf中
    strcpy(nameBuf, recBuf); // 这里其实最好要考虑线程并发对recBuf值的改变，可以考虑使用互斥量等方法
    pthread_t tid;
    tid = pthread_self();
    printf("启动线程tid:%lu,用于接收新蓝牙从机%s的信息\n", tid, nameBuf);
    int BTlen = -1;
    char buff[16432];
    char buff2[16432];

    pthread_attr_t BTconnid_attr; // 创建线程属性
    pthread_attr_init(&BTconnid_attr);
    pthread_attr_setstacksize(&BTconnid_attr, 1024 * 1024); // 设置栈大小 <1MB
    pthread_t BTconnid;                                     // 监听心跳包
    pthread_create(&BTconnid, &BTconnid_attr, (void *)BTconn_listen, NULL);
    printf("启动线程tid:%lu,监听心跳包线程:%lu\n", tid, BTconnid);

    while (1)
    {
        memset(readBuf, 0, 16432);
        n_read = read(tmp_c_fd, buff, sizeof(buff)); // read是阻塞接收
        BTlen = n_read;
        if (n_read <= 0)
        {
            perror("read"); // 调试语句
            printf("%s中断或者下线了 %d\n", nameBuf, tmp_c_fd);
            tmp_c_fd = -1;     // 如果对应的客户端退出,则令对应的c_fd的值为-1,表示掉线
            recvheartflag = 1; // 告诉监听心跳线程 让他结束。
            pthread_join(BTconnid, NULL);
            printf("连接断开，监听心跳包线程结束\n");
            pthread_exit(NULL); // 如果客户端掉线，结束线程
        }
        if (BTlen > 0)
        {
            uintptr_t ptr2 = (uintptr_t)buff;
            sk_head *head2 = (sk_head *)ptr2;
            int size = head2->size + 16 + 1;
            if (BTlen > size)
            {
                int offset = 0;
                while (offset < BTlen)
                {
                    sk_head *current_head = (sk_head *)(buff + offset); // 获取当前指令的头部
                    int current_size = current_head->size + 16 + 1;     // 计算当前指令的总长度
                    char buffer[current_size];
                    memcpy(buffer, buff + offset, current_size); // 将当前指令+标志位+复制到新的缓冲区
                    int states;
                    if (buffer[current_size - 1] == '6')
                    {
                        states = send(fd_svr, buffer, current_size - 1, 0);
                    }
                    else if (buffer[current_size - 1] == '7')
                    {
                        states = send(fd_relay, buffer, current_size - 1, 0);
                    }
                    else if (buffer[current_size - 1] == '9')
                    {
                        printf("蓝牙收到心跳包\n");
                    }
                    else // 出现了粘包现象。两个指令连在一起只有最后一位标志位
                    {
                        current_size = current_size - 1;
                        states = send(fd_relay, buffer, current_size, 0);
                    }
                    // 处理发送成功还是失败
                    if (states >= 0)
                    {
                        printf("send %d to socket 17001 socket %d\n", BTlen, states);
                        // memset(buff, 0, sizeof(buff));
                    }
                    else if (states == -1)
                    {
                        if (errno == EPIPE)
                        {
                            printf("Client socket has been closed.\n");
                            // break;
                        }
                        else
                        {
                            printf(" 167001 Send failed with error: %s\n", strerror(errno));
                        }
                    }
                    offset += current_size; // 更新偏移量以处理下一条指令
                }
                memset(buff, 0, sizeof(buff)); // 在处理完所有指令后清空buf
            }
            else if (BTlen == size)
            {
                printf("usblen %d == size :%d\n", BTlen, size);
                //  发送TCP消息
                if (buff[BTlen - 1] == '6')
                {
                    buff[BTlen - 1] = end;
                    uintptr_t ptr = (uintptr_t)buff;
                    sk_head *head = (sk_head *)ptr;
                    memcpy(buff2, &buff[sizeof(s)], BTlen - sizeof(s));
                    printf("read data from 16001 BT:%s\n", buff2);
                    int states = send(fd_svr, buff, BTlen - 1, 0);
                    if (states >= 0)
                    {
                        printf("send  %s  to socket 16001\n", buff2);
                        memset(buff, 0, sizeof(buff));
                    }
                    else if (states == -1)
                    {
                        if (errno == EPIPE)
                        {
                            printf("Client %d socket has been closed.\n", fd_svr);
                            // break;
                        }
                        else
                        {
                            printf("16001 Send failed with error: %s\n", strerror(errno));
                        }
                    }
                }
                else if (buff[BTlen - 1] == '7')
                {
                    buff[BTlen - 1] = end;
                    uintptr_t ptr = (uintptr_t)buff;
                    sk_head *head = (sk_head *)ptr;
                    memcpy(buff2, &buff[sizeof(s)], sizeof(buff) - sizeof(s));
                    //  发送TCP消息
                    printf("read %d data from 17001 BT:%s\n", BTlen, buff2);
                    int states = send(fd_relay, buff, BTlen - 1, 0);
                    if (states >= 0)
                    {
                        printf("send %d to socket 17001 socket %d\n", BTlen, states);
                        memset(buff, 0, sizeof(buff));
                    }
                    else if (states == -1)
                    {
                        if (errno == EPIPE)
                        {
                            printf("Client %d socket has been closed.\n", fd_relay);
                            // break;
                        }
                        else
                        {
                            printf("17001 Send failed with error: %s\n", strerror(errno));
                        }
                    }
                }
                else if (buff[BTlen - 1] == '8') // 新指令
                {
                    usbreconnect = 1;
                    buff[BTlen - 1] = end;
                    uintptr_t ptr = (uintptr_t)buff;
                    sk_head *head = (sk_head *)ptr;
                    memcpy(buff2, &buff[sizeof(s)], BTlen - sizeof(s));
                    printf("read data from 16001 BT:%s\n", buff2);

                    int states = send(fd_svr, buff, BTlen - 1, 0);
                    if (states >= 0)
                    {
                        printf("send %s  to socket 16001\n", buff2);
                        memset(buff, 0, sizeof(buff));
                    }
                    else if (states == -1)
                    {
                        if (errno == EPIPE)
                        {
                            printf("Client %d socket has been closed.\n", fd_svr);
                            // break;
                        }
                        else
                        {
                            printf("Send failed with error: %s\n", strerror(errno));
                        }
                    }
                }
                else if (buff[BTlen - 1] == '9') // 连接心跳
                {
                    printf("BT receive heartbeat\n");
                }
                else
                {
                    int states = send(fd_svr, buff, BTlen, 0);
                    if (states >= 0)
                    {
                        printf("2 send %s  to socket 16001\n", buff2);
                        memset(buff, 0, sizeof(buff));
                    }
                }
            }
        }
    }
    printf("bluetooth recv closed\n");
}

int acc = 0; // 发送失败断开16001
int svr_flag = 0;
int svr_start()
{
    while (1)
    {
        if (USBorBTflag == 0) // usb flag
        {
            if (abc == 0) // usb连接上了,收到心跳包了
            {
                tcp_server = socket(AF_INET, SOCK_STREAM, 0);
                struct sockaddr_in tcp_addr;
                tcp_addr.sin_family = AF_INET;
                tcp_addr.sin_port = htons(16001);
                tcp_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
                int optval = 1;
                if (setsockopt(tcp_server, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1)
                {
                    perror("usb 16001setsockopt");
                    return 1;
                }
                bind(tcp_server, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr));
                listen(tcp_server, 1);
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                printf("usb TCP server  16001 已启动,等待连接...\n");
                fd_svr = accept(tcp_server, (struct sockaddr *)&client_addr, &client_len);
                pthread_t svrid;
                pthread_create(&svrid, NULL, (void *)svr_process, (void *)fd_svr);
                printf("usb 16001 连接成功...fd_svr== %d \n", fd_svr);
            }
            while (1)
            {
                if (abc == 1) // 断开连接了//
                {
                    if (fd_svr > 0)
                    {
                        printf("usb 16001 断开连接...fd_svr== %d \n", fd_svr);
                        close(fd_svr); // 关闭客户端连接
                        fd_svr = -1;
                    }
                    if (tcp_server > 0)
                    {
                        printf("BT 16001 断开连接...tcp_server== %d \n", tcp_server);
                        close(tcp_server); // 关闭服务器socket
                        tcp_server = -1;
                    }
                    usleep(1000 * 500);
                    break;
                }
                usleep(200 * 1000);
            }
        }
        if (USBorBTflag == 1) // bluetooth
        {
            if (babc == 0 || acc == 1) // usb连接上了,收到心跳包了
            {
                acc = 0;
                tcp_server = socket(AF_INET, SOCK_STREAM, 0);
                struct sockaddr_in tcp_addr;
                tcp_addr.sin_family = AF_INET;
                tcp_addr.sin_port = htons(16001);
                tcp_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
                int optval = 1;
                if (setsockopt(tcp_server, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1)
                {
                    perror("BT 16001setsockopt");
                    return 1;
                }
                bind(tcp_server, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr));
                listen(tcp_server, 1);
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                printf("BT TCP server  16001 已启动,等待连接...\n");
                fd_svr = accept(tcp_server, (struct sockaddr *)&client_addr, &client_len);

                /**********************************************************************/
                pthread_attr_t svrid_attr; // 创建线程属性
                pthread_attr_init(&svrid_attr);
                pthread_attr_setstacksize(&svrid_attr, 1024 * 1024); // 设置栈大小 <1MB
                pthread_t svrid;
                pthread_create(&svrid, &svrid_attr, (void *)svr_process, (void *)fd_svr);
                pthread_detach(svrid);
                printf("启动16001 数据转发线程\n");
                /**********************************************************************/
            }
            while (1)
            {
                if (babc == 1 || acc == 1) // babc 蓝牙断开
                {
                    if (fd_svr > 0)
                    {
                        printf("BT 16001 断开连接...fd_svr== %d \n", fd_svr);
                        close(fd_svr); // 关闭客户端连接
                        fd_svr = -1;
                    }
                    if (tcp_server > 0)
                    {
                        printf("BT 16001 断开连接...tcp_server== %d \n", tcp_server);
                        close(tcp_server); // 关闭服务器socket
                        tcp_server = -1;
                    }
                    usleep(1000 * 500);
                    break;
                }
                usleep(200 * 1000);
            }
        }
    }
    return 0;
}

static void svr_process(void *fd)
{
    int conn = (int)fd;
    printf("enter 16001 socket number %d\n", conn);
    int i;
    ssize_t TCPlen;
    char buf[2049];
    char buf2[2049];
    char flags = 's';
    char flagg = 'g';
    char flag = '6';
    int abcd = 0;
    while (1)
    {
        // 接收TCP消息
        int TCPlen = recv(conn, buf, sizeof(buf), MSG_DONTWAIT); // MSG_DONTWAIT
        //  发送USB消息
        if (TCPlen > 0)
        {
            uintptr_t ptr2 = (uintptr_t)buf;
            sk_head *head2 = (sk_head *)ptr2;
            memcpy(buf2, &buf[sizeof(s2)], sizeof(buf) - sizeof(s2));
            printf("16001 recv tcp %s\n", buf2);
            buf[TCPlen] = flags; // 在字符串最后添加标志位
            buf[TCPlen + 1] = flags;
            buf[TCPlen + 2] = flagg;
            buf[TCPlen + 3] = flag;
            buf[TCPlen + 4] = '\0';
            if (USBorBTflag == 0) // USB连接,转发消息
                abcd = write(usb, buf, TCPlen + 4);
            else // 蓝牙连接，转发消息
                abcd = write(bt_fd, buf, TCPlen + 4);
            if (abcd == -1) // usb发送失败，说明也是断开了
            {
                errorflag = 1;
                printf("16001 Send to usb %d failed with error: %s\n", usb, strerror(errno));
                acc = 1; // 发生了错误
                break;
            }
            memset(buf, 0, sizeof(buf));
        }
        if (errorconsvr == 1 || acc == 1)
        {
            printf("errorconsvr 16001 \n");
            break;
        }
        usleep(1000);
    }
    close(conn);
    printf("close 16001 socket  %d \n", conn);
    pthread_exit(NULL);
}

int relay_start()
{

    int result;
    struct sockaddr_in tcp_addr;
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_port = htons(17001);
    tcp_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    while (1)
    {
        relay_server = socket(AF_INET, SOCK_STREAM, 0);
        if (relay_server == -1)
        {
            perror("socket");
            sleep(1);
            continue;
        }

        int optval = 1; // 防止应用程序在短时间内被终止并重新启动时出现“地址已在使用”错误
        if (setsockopt(relay_server, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1)
        {
            perror("17001setsockopt");
            sleep(1);
            continue;
        }

        result = bind(relay_server, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr));
        if (result == -1)
        {
            perror("bind");
            close(relay_server);
            sleep(1);
            continue;
        }

        result = listen(relay_server, 1);
        if (result == -1)
        {
            perror("listen");
            close(relay_server);
            sleep(1);
            continue;
        }

        break;
    }
    pthread_attr_t threadid2_attr; // 创建线程属性
    pthread_attr_init(&threadid2_attr);
    pthread_attr_setstacksize(&threadid2_attr, 1024 * 1024); // 设置栈大小 <1MB
    pthread_t threadid2;
    pthread_create(&threadid2, &threadid2_attr, (void *)relay_listen, NULL);
    printf("TCP 17001  启动监听连接线程\n");
    return threadid2 ? 1 : 0;
}

static void relay_listen(void)
{

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    while (1)
    {
        printf("TCP server 17001  已启动,等待连接...\n");
        fd_relay = accept(relay_server, (struct sockaddr *)&client_addr, &client_len);
        if (-1 != fd_relay)
        {
            printf("TCP server 17001  已连接!!!!\n");
            pthread_attr_t relayid_attr; // 创建线程属性
            pthread_attr_init(&relayid_attr);
            pthread_attr_setstacksize(&relayid_attr, 1024 * 1024); // 设置栈大小 <1MB
            pthread_t relayid;
            pthread_create(&relayid, NULL, (void *)relay_process, (void *)fd_relay);
            pthread_detach(relayid); // 将线程设置为分离状态，线程结束后，其资源会自动释放。
            printf("TCP server 17001  已启动数据转发线程\n");
        }
        else
        {
            perror("17001accept");
            usleep(10 * 1000 * 1000);
        }
    }
}

int COUNT = 0;
static void relay_process(void *fd)
{
    usbreconnect = 0;
    int conn = (int)fd;
    printf("enter 17001 socket number %d\n", conn);
    ssize_t TCPlen;
    int USBlen = -1;
    char buf[16432 * 10]; //[1024 * 16 + 48 + 10];//空间是富于的
    char buf2[2049];
    char flagg = 'g';
    char flags = 's';
    char flag = '7';
    int count = 1;
    int abcd = 0;

    while (1)
    {
        TCPlen = recv(conn, buf, sizeof(buf), MSG_DONTWAIT); // MSG_DONTWAIT

        if (TCPlen > 0) // 返回-1 接收失败。返回0 对端关闭
        {
            uintptr_t ptr2 = (uintptr_t)buf;
            sk_head *head2 = (sk_head *)ptr2;
            int size = head2->size + 16;
            printf("size %d   TCPlen %d \n", size, TCPlen);
            if (TCPlen == size)
            {
                buf[TCPlen] = flags; // 在字符串最后添加标志位
                buf[TCPlen + 1] = flags;
                buf[TCPlen + 2] = flagg;
                buf[TCPlen + 3] = flag;
                buf[TCPlen + 4] = '\0';
                if (USBorBTflag == 1) // 蓝牙连接
                {
                    // if (TCPlen != 16432)
                    usleep(1000);
                    abcd = send(bt_fd, buf, TCPlen + 4, 0);
                }
                else
                {
                    abcd = write(usb, buf, TCPlen + 4);
                }
                if (abcd < 0)
                {
                    errorflag = 1;
                    printf("== usb Send failed with error: %s\n", strerror(errno));
                    break;
                }
                else
                    printf("== 17001 write  to usb %d \n", abcd);
            }
            else if (TCPlen > size)
            {
                int offset = 0;
                while (offset < TCPlen)
                {
                    sk_head *current_head = (sk_head *)(buf + offset); // 获取当前指令的头部
                    int current_size = current_head->size + 16;        // 计算当前指令的总长度
                    char buffer[current_size + 4];
                    if (offset + current_size > TCPlen)
                    {
                        memcpy(buffer, buf + offset, TCPlen - offset);
                        char buf4[offset + current_size - TCPlen];
                        int TCPlen3 = recv(conn, buf4, sizeof(buf4), 0); // 阻塞接收
                        memcpy(buffer, buf4, sizeof(buf4));              // 将当前指令复制到新的缓冲区
                    }
                    else
                        memcpy(buffer, buf + offset, current_size); // 将当前指令复制到新的缓冲区

                    // 添加标志位
                    buffer[current_size] = flags; // 在字符串最后添加标志位
                    buffer[current_size + 1] = flags;
                    buffer[current_size + 2] = flagg;
                    buffer[current_size + 3] = flag;
                    buffer[current_size + 4] = '\0';
                    if (USBorBTflag == 0) // USB 连接
                        abcd = write(usb, buffer, current_size + 4);
                    else // 蓝牙连接
                    {
                        usleep(1000);
                        // if (current_size != 16432)
                        abcd = write(bt_fd, buffer, current_size + 4);
                    }
                    if (abcd == -1)
                    {
                        errorflag = 1;
                        printf("zhaobao usb Send failed with error: %s\n", strerror(errno));
                    }
                    else
                    {
                        printf("zhanbao 17001 write  to usb %d \n", abcd);
                    }
                    offset += current_size; // 更新偏移量以处理下一条指令
                }
            }
            else //(TCPlen < size)
            {
                char buf3[size - TCPlen];
                int TCPlen2 = recv(conn, buf3, sizeof(buf3), 0);
                memcpy(buf + TCPlen, buf3, sizeof(buf3));
                buf[TCPlen] = flags; // 在字符串最后添加标志位
                buf[TCPlen + 1] = flags;
                buf[TCPlen + 2] = flagg;
                buf[TCPlen + 3] = flag;
                buf[TCPlen + 4] = '\0';
                // int abcd = write(bt_fd, buf, TCPlen + TCPlen2 + 4);

                if (USBorBTflag == 0) // USB 连接
                    abcd = write(usb, buf, TCPlen + TCPlen2 + 4);
                else // 蓝牙连
                {
                    // if (TCPlen + TCPlen2 != 16432)
                    usleep(1000);
                    abcd = write(bt_fd, buf, TCPlen + TCPlen2 + 4);
                }
                if (abcd == -1)
                {
                    printf("<< usb Send failed with error: %s\n", strerror(errno));
                    break;
                }
                else
                {
                    printf(" << 17001 write  to usb %d \n", abcd);
                }
            }
        }
        if (usbreconnect == 1 || errorconrel == 1)
        {
            if (usbreconnect == 1)
            {
                printf("17001断开!!!\n");
            }
            if (errorconrel == 1)
            {
                printf("errorconrel 17001 \n");
            }
            break;
        }
        usleep(10);
    }
    close(conn);
    printf("17001线程 close\n");
    pthread_exit(NULL);
}

/**
 * @brief 18001转发连接状态
 */
int heart_start()
{
    int result;
    struct sockaddr_in tcp_addr;

    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_port = htons(18001);
    tcp_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    while (1)
    {
        heart_server = socket(AF_INET, SOCK_STREAM, 0);
        if (heart_server == -1)
        {
            perror("socket");
            sleep(1);
            continue;
        }

        int optval = 1;
        if (setsockopt(heart_server, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1)
        {
            perror("setsockopt");
            close(heart_server);
            sleep(1);
            continue;
        }

        result = bind(heart_server, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr));
        if (result == -1)
        {
            perror("bind");
            close(heart_server);
            sleep(1);
            continue;
        }

        result = listen(heart_server, 1);
        if (result == -1)
        {
            perror("listen");
            close(heart_server);
            sleep(1);
            continue;
        }

        break;
    }
    pthread_attr_t threadid3_attr; // 创建线程属性
    pthread_attr_init(&threadid3_attr);
    pthread_attr_setstacksize(&threadid3_attr, 1024 * 1024); // 设置栈大小 <1MB
    pthread_t threadid3;
    pthread_create(&threadid3, &threadid3_attr, (void *)heart_listen, NULL);
    printf("TCP server 18001  启动监听线程\n");
    return threadid3 ? 1 : 0;
}

int connflag18001 = 0; // 0-没有连接18001
static void heart_listen(void)
{

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    while (1)
    {
        printf("TCP server 18001  已启动,等待连接...\n");
        fd_heart = accept(heart_server, (struct sockaddr *)&client_addr, &client_len); // accept阻塞接收啊？那为什么会出现
        // 一直打印TCP server 18001  已启动,等待连接...是因为accept失败，会赋值为-1 ,然后就会一直的去循环这个过程
        if (-1 != fd_heart)
        {
            printf("TCP server 18001  已连接!!!!\n");
            connflag18001 = 1;
        }
        else
        {
            perror("accept");
            usleep(10 * 1000 * 1000);
        }
        usleep(100 * 1000);
    }
}

/*监听状态线程
 */
int retflag = -1; // 判断发送给APP 指令是否相同、、相同就补重复发送了
void send_conn_status(int connflag)
{
    char charconnflag[10];

    if (retflag == connflag)
        return;

    if (connflag == 0)
        strcpy(charconnflag, "usb");
    else if (connflag == 1)
        strcpy(charconnflag, "bluetooth");
    else if (connflag == 2)
        strcpy(charconnflag, "gggg");

    if (connflag18001 == 1) // 连接上了
    {
        while (1)
        {
            int ret = send(fd_heart, &charconnflag, sizeof(charconnflag), 0);
            if (ret <= 0)
            {
                perror("18001 send");
                printf("Waiting for connection...\n");
                sleep(1);
            }
            else
            {
                printf("send conn status to 18001  %s \n", charconnflag);
                break;
            }
            usleep(1000);
        }
    }
    else
        printf("18001 TCP no connection!\n");
}

/**
 *@brief  返回相差us数
 *@param  两个比较的struct timeval结构体
 *@return 失败返回-1；成功返回0；

 gettimeofday(&tv, NULL);//获取当前时间
            time_gap = GetTimeInterval(start_tv, stop_tv);
            //printf("time_gap = %d\n", time_gap);
 */
static unsigned int GetTimeInterval(struct timeval src_tv, struct timeval dest_tv)
{
    unsigned int TimeUs;
    struct timeval start;
    struct timeval stop;

    if (src_tv.tv_sec < dest_tv.tv_sec)
    {
        memcpy(&start, &src_tv, sizeof(start));
        memcpy(&stop, &dest_tv, sizeof(stop));
    }
    else if (src_tv.tv_sec > dest_tv.tv_sec)
    {
        memcpy(&start, &dest_tv, sizeof(start));
        memcpy(&stop, &src_tv, sizeof(stop));
    }
    else /*sec ==*/
    {
        if (src_tv.tv_usec < dest_tv.tv_usec)
        {
            memcpy(&start, &src_tv, sizeof(start));
            memcpy(&stop, &dest_tv, sizeof(stop));
        }
        else
        {
            memcpy(&start, &dest_tv, sizeof(start));
            memcpy(&stop, &src_tv, sizeof(stop));
        }
    }

    if (stop.tv_sec == start.tv_sec)
    {
        TimeUs = stop.tv_usec - start.tv_usec;
    }
    else if (stop.tv_sec > start.tv_sec)
    {
        if (stop.tv_usec < start.tv_usec)
        {
            TimeUs = (stop.tv_sec - 1 - start.tv_sec) * 1000000 + (stop.tv_usec + 1000000 - start.tv_usec);
        }
        else
        {
            TimeUs = (stop.tv_sec - start.tv_sec) * 1000000 + (stop.tv_usec - start.tv_usec);
        }
    }
    else
    {
        TimeUs = 0;
    }
    return TimeUs;
}

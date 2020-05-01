# opencv_ffmpeg_streaming
rtmp streaming from opencv with ffmpeg / avcodec

Using ffmpeg libraries from C/C++ is tricky and I could not easily find easy examples without memory leaks or bad crashes that used OpenCV as input or for image processing.
This is an example for using the ffmpeg and opencv libraries from C++ for rtmp streaming. 
The input is from the OpenCV VideoCapture class.
To build Release:
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
```

To build Debug:
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Debug ..
$ make
```
In main.cpp the stream url is defined as "rtmp://localhost/live/mystream" and must be adapted to your rtmp server settings

Run the program on a video:
```
$ ./build/simple_opencv_streaming <video_file>
```
From camera
```
$ ./build/simple_opencv_streaming 0
```

To convert the rtmp stream to HLS and publish the stream to a browser I have written a tutorial on  <a href="https://www.nobile-engineering.com/wordpress/index.php/2018/10/30/video-streaming-hls-apache-nginx/"> a blog post </a> 

## Stream Application and Stream Name
- To define the stream application and stream name, change the parameter in the `StreamConfig`, here we use `rtmp://localhost/zed_left_camera/stream`. the `zed_left_camera` is a stream application name, the `stream` is the stream name.
- Attention, the stream application should be the same with the application name in the RTMP server (the ngnix RTMP server in our case).
- Once the programe is launched, you can always see the stream without much latency by the command `ffplay -fflags nobuffer rtmp://localhost/zed_left_camera/stream` (wait for several seconds), this is very useful to debug.
- You can also test the rtmp link in the browser (need flash player support), we have used `https://www.wowza.com/testplayers` for example.

## Latency problem
- To test the latency of the server side, use `ffplay -fflags nobuffer rtmp://localhost/zed_left_camera/stream`
- To reduce the server side latency, you can only change the interval frame's number (`codec_ctx->gop_size` in our case). For 30 fps video, `gop_size=12` means `0.4s` latency in the server side. The smaller interval frame, the smaller latency, but consuming more bandwidth.
- Most of the latency is introduced in the client side, usually to keep the smoothness, the clients buffer some frame, and thus the latency largely incresed. For the web flash player, the average buffer time is about 1.5 seconds, which may be changed by the different buffering settings.

## Install and configure Ngnix with RTMP module
### Install Ngnix with RTMP module
- For the details, see https://github.com/arut/nginx-rtmp-module/wiki/Getting-started-with-nginx-rtmp
- This is a summary
    - download nginx 1.6  (latest stable version)
    - download nginx-rtmp-module
    - run `./configure --with-http_ssl_module --add-module=../nginx-rtmp-module`, `make`, `sudo make install`.

### Start the server

- Change the `/usr/local/nginx/conf/nginx.conf`, you can find the file that we use in `nginx-rtmp-related/nginx.conf`.
    - In this configuration, our live appliaction is called "zed_left_camera".
- Add nginx to upstart. Save lines below in the new file `/etc/init/nginx.conf` (you can find more details in https://serverfault.com/questions/143461/how-can-i-start-nginx-via-upstart)
    ```
    description "nginx http daemon"

    start on (filesystem and net-device-up IFACE=lo)
    stop on runlevel [!2345]

    env DAEMON=/usr/local/nginx/sbin/nginx
    env PIDFILE=/var/run/nginx.pid

    # Needed to allow Nginx to start, however, the wrong PID will be tracked
    expect fork

    # Test the nginx configuration (Upstart will not proceed if this fails)
    pre-start exec $DAEMON -t

    # Ensure nginx is shutdown gracefully
    # Upstart will be tracking the wrong PID so the following is needed to stop nginx
    post-stop exec start-stop-daemon --stop --pidfile $PIDFILE --name nginx --exec $DAEMON --signal QUIT

    # Start Nginx
    exec $DAEMON
    ```

- Start the server by `sudo /usr/local/nginx/sbin/nginx`
- Stop the server by `sudo /usr/local/nginx/sbin/nginx -s stop`




## Add Hardware Support
- If you have not nvidia graphic card, you can just replace `out_codec = avcodec_find_encoder_by_name("h264_nvenc");` by `out_codec = avcodec_find_encoder(codec_id);`, the latter one will use only the CPU.
- But if you have nvidia graphic card, use hardware acceleration will largely decrease the usage of CPU, which is incredibly important in robotic.

### Nvidia
- install `nv-codec-headers`, which is FFmpeg own slightly modified runtime-loader for nvidias CUDA/nvenc/nvdec related libraries. **Don't forget to checkout to sdk/8.0**, otherwise the nvidia driver will blame you that you need to upgrade it.

    ```[bash]
        git clone https://git.videolan.org/git/ffmpeg/nv-codec-headers.git
        cd nv-codec-headers
        git checkout sdk/8.0
        sudo make install PREFIX=/usr
    ```

  - check `/usr/lib/pkgconfig` is in the `PKG_CONFIG_PATH`. (use `echo $PKG_CONFIG_PATH`). If not, add `PKG_CONFIG_PATH=/usr/lib/pkgconfig:$PKG_CONFIG_PATH` to `~/.bashrc` and update the environment variables by `source ~/.bashrc`.

  - For more details, see https://superuser.com/questions/1299064/error-cuvid-requested-but-not-all-dependencies-are-satisfied-cuda-ffnvcodec

- Compile `ffmpeg` 
    - command
        ```
        ./configure --enable-cuda --enable-cuvid --enable-nvenc
        make -j 4
        sudo make install
        ```
    - for more details, see https://superuser.com/questions/1299064/error-cuvid-requested-but-not-all-dependencies-are-satisfied-cuda-ffnvcodec 


- compile this project:
    - You can compile easily with this command: (for more details, see https://stackoverflow.com/questions/51384563/ffmpeg-avcodec-find-encoder-by-name-failes-to-find-encoder-h264-nvenc)

    ```
        mkdir build
        cd build
        g++ ../main.cpp ../streamer/streamer.cpp `pkg-config --cflags libavformat libswscale libswresample libavutil libavcodec opencv` `pkg-config --libs libavformat libswscale libswresample libavutil libavcodec opencv` -std=gnu++11 -o streamer
    ```

    - You can always find the linked library by the command `ldd ./streamer`

    - However, things become really complicated when you want to use CMakelist (which is necessary to build a ROS node, for example), here we use a hack technic:
        - run  `pkg-config --libs libavformat libswscale libswresample libavutil libavcodec` in terminal to find the needed library
        - copy and past then in CmakeLists.txt, for example `set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -L/usr/local/lib -lavcodec -llzma -lz -lswresample  -lavutil -pthread -lm -ldl")`

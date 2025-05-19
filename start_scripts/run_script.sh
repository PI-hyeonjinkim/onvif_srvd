#!/bin/sh

DAEMON=onvif_srvd_debug
DAEMON_PATH=.
PID_FILE=$DAEMON.pid

DAEMON_ARGS="--pid_file $PID_FILE"
DAEMON_ARGS="$DAEMON_ARGS --ifs eth0"
DAEMON_ARGS="$DAEMON_ARGS --scope onvif://www.onvif.org/name/Jetson"
DAEMON_ARGS="$DAEMON_ARGS --scope onvif://www.onvif.org/location/Lab"
DAEMON_ARGS="$DAEMON_ARGS --scope onvif://www.onvif.org/Profile/Streaming"
DAEMON_ARGS="$DAEMON_ARGS --scope onvif://www.onvif.org/Profile/S"
DAEMON_ARGS="$DAEMON_ARGS --name JetsonCam"
DAEMON_ARGS="$DAEMON_ARGS --width 1280 --height 720"
DAEMON_ARGS="$DAEMON_ARGS --url rtsp://192.168.33.13:8554/mystream"
DAEMON_ARGS="$DAEMON_ARGS --type H264"

d_start()
{
    if [ -f $PID_FILE ] && kill -0 $(cat $PID_FILE); then
        echo "$DAEMON already running"
        return 1
    fi

    echo "Starting $DAEMON..."
    $DAEMON_PATH/$DAEMON $DAEMON_ARGS && echo "$DAEMON started"
}

d_stop()
{
    if [ ! -f "$PID_FILE" ] || ! kill -0 $(cat "$PID_FILE"); then
        echo "$DAEMON not running"
        return 1
    fi

    echo "Stopping $DAEMON..."
    kill -15 $(cat $PID_FILE) && rm -f $PID_FILE
    echo "$DAEMON stopped"
}

case "$1" in
      start)
          d_start
          ;;
      stop)
          d_stop
          ;;
      restart)
          echo "Restarting $DAEMON"
          d_stop
          d_start
          ;;
      *)
          echo "Usage: $0 {start|stop|restart}"
          exit 1
          ;;
esac
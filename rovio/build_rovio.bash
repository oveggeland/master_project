if [ -z "$1" ]
then
	catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DROVIO_NCAM=2
else
	catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DROVIO_NCAM=$1
fi

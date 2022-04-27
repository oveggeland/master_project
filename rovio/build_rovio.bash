if [ -z "$1" ]
then
	mkdir halla
	catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DROVIO_NCAM=2
else
	mkdir hei
	catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DROVIO_NCAM=$1
fi

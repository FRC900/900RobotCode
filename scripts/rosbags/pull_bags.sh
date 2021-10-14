JETSON_ADDR=10.9.0.8

source ~/.bashrc
mkdir -p bagfiles/
scp ubuntu@$JETSON_ADDR:/mnt/900_2/_2019-* bagfiles/
./safe_search.sh bagfiles/_2019-*


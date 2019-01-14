from acfr/ros-build:kinetic

# Add in the scripts
ADD scripts/* /opt/bagdb/

# Add in the dependencies
RUN wget -O - http://acfr-ros-pro-1.srv.sydney.edu.au:4440/acfr_its.gpg.key | apt-key add - && \
echo "deb http://acfr-ros-pro-1.srv.sydney.edu.au:4440/zio-dev xenial main" > /etc/apt/sources.list.d/zio.list && \
apt-get update && \
apt-get -y install vim nano python-utm python-psycopg2 python-shapely && \
apt clean all

# Add the scripts to the path
ENV PATH=/opt/bagdb:$PATH

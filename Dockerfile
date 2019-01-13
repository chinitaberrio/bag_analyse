from acfr/ros-build:kinetic

# Add in the scripts
ADD scripts/* /opt/bagdb

# Add in the dependencies
RUN apt-get update && apt-get -y install python-utm python-psycopg2 python-shapely

# Add the scripts to the path
ENV PATH=/opt/bagdb:$PATH

cmake_minimum_required(VERSION 2.8.3)
project(hekateros_rosbridge)

INSTALL(DIRECTORY www/img DESTINATION /opt/nginx/html/ros)
INSTALL(DIRECTORY www/js DESTINATION /opt/nginx/html/ros)
INSTALL(FILES www/html/ros/hekateros/index.html 
        DESTINATION /opt/nginx/html/ros/hekateros)


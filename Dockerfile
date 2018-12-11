From argnctu/sis_base_image:v4


WORKDIR /root/sis_mini_competition_2018/
RUN rm -rf ./catkin_ws/src/competition_modules

COPY competition_modules/ ./catkin_ws/src
COPY master_task.launch ./catkin_ws/src/sis_arm/sis_arm_planning/launch/

COPY run_task.sh .
RUN /bin/bash -c "cd ~/sis_mini_competition_2018/ && source /opt/ros/kinetic/setup.bash && catkin_make -C catkin_ws/"
RUN /bin/bash -c "source ~/sis_mini_competition_2018/catkin_ws/devel/setup.bash"


CMD [ "./run_task.sh" ] 

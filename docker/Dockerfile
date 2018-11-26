From argnctu/sis_base_image:v1


WORKDIR ~/sis_mini_competition_2018

COPY ./../competition_modules ./catkin_ws/src
COPY ./../master_task.launch ./catkin_ws/src/sis_arm/sis_arm_planning/launch

COPY ./run_task.sh .
RUN /bin/bash -c "cd ~/sis_mini_competition_2018/ && source /opt/ros/kinetic/setup.bash && catkin_make -C catkin_ws/"

CMD [ "./run_task.sh" ] 

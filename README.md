# sis_competition_task_template

## About this template

You need to submit by using this template for each task.

## Path:

> competition_modules

>> object_detection
    
>>> src (Put your source code here)
    
>>> srv (Design a service used for the task)
    
>>> launch (put your launch file here)
    
>> place_to_box
    
>>> src (Put your source code here)
    
>>> srv (Design a service used for the task)
    
>>> launch (put your launch file here)
    
>> pose_estimate_and_pick
    
>>> src (Put your source code here)
    
>>> srv (Design a service used for the task)
    
>>> launch (put your launch file here)
    
>> robot_navigation
    
>>> src (Put your source code here)
    
>>> srv (Design a service used for the task)
    
>>> launch (put your launch file here)
              
> README.md

> Dockerfile            (You don't need to modify this file)

> run_task.sh           (You don't need to modify this file)

> master_task.launch    (You have to determine which node you need to launch and write in this file)

> docker_build.sh       (If you want to build docker file, please execute/source this shell)


## How to build docker image:

tx2 $ source docker_build.sh

***If docker is already login with other account, please logout first.***

tx2 $ docker logout

***Type your dockerhub's account and password.***

tx2 $ docker login

tx2 $ docker tag sis_competition [dockerhub account]/sis_competition:[task_name]

tx2 $ docker push [dockerhub account]/sis_competition:[task_name]

  

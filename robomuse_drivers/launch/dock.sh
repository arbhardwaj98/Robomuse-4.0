cd 
echo "Moving to point"
rosrun robomuse_drivers docknewphase1.cpp
sleep 3
echo "Docking"
rosrun robomuse_drivers dock_phase2
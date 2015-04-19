// reachability_from_above.cpp
// wsn, March 2015
// compute reachability, w/ z_tool_des = [1;0;0] = x_base
// w/ robot mounted as is, x_base points down
// Fix the value of x, --> const height; scan over y and z
//const double x_des = 0.374;

#include <ros/ros.h>
#include <irb120_kinematics.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <fstream>
//#include <std_msgs/Int16MultiArray>

int main(int argc, char **argv) {
    ros::init(argc, argv, "irb120_reachability");
    // ros::NodeHandle nh;

    // ros::Publisher reachPtPub = nh.advertise<geometry_msgs::Point>( "reachablePt", 1 ); // add a publisher
    // ros::Publisher bestIkSoluNumPub = nh.advertise<std_msgs::Int16>( "bestIkSoluNum", 1 ); // add a publisher
    // ros::Publisher reachPtPub = nh.advertise<std_msgs/Int16MultiArray>( "reachablePt",1 ); // add a publisher
    // ros::Publisher ikSoluNumPub = nh.advertise<std_msgs/Int16MultiArray>( "ikSoluNum", 1 ); // add a publisher
    // ros::Rate timer(1); // sleep timer for 5Hz repetition rate

    // create a variable of type "Point" to publish the desired point for manipulator's tool flange to move
    geometry_msgs::Point desPt;
    desPt.x = 0.0;
    desPt.y = 0.0;
    desPt.z = 0.0;

    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    Vectorq6x1 q_in;
    q_in<<0,0,0,0,0,0;

    Irb120_fwd_solver irb120_fwd_solver;
    Irb120_IK_solver ik_solver;

    b_des<<1,0,0;
    t_des<<0,1,0;
    n_des = t_des.cross(b_des);
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    
    //need too transform to convert between DH and URDF frames
    /* Eigen::Affine3d a_tool;
    a_tool.linear() =R_des;
    a_tool.translation() << 0.3,
            0.0,
            0.3;

    
    Eigen::Affine3d A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(q_in); //fwd_kin_solve
    // rotate DH frame6 to reconcile with URDF frame7:
    //Eigen::Affine3d A_fwd_URDF = A_fwd_DH*a_tool;
    std::cout << "q_in: " << q_in.transpose() << std::endl;
    std::cout << "A rot: " << std::endl;
    std::cout << A_fwd_DH.linear() << std::endl;
    std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;
    
    int nsolns = ik_solver.ik_solve(A_fwd_DH);*/
    std::vector<Vectorq6x1> q6dof_solns;
    // std::vector<Eigen::Vector3d> reachPtVec;
    //std::vector<Vector3d> reachPt_X_Vec;
    //std::vector<Vector3d> reachPt_Y_Vec;
    //std::vector<Vector3d> reachPt_Z_Vec;


    /*// Vectorq6x1 qsoln;
    ik_solver.get_solns(q6dof_solns);
    std::cout<<"IK solns: "<<std::endl;
    for (int i=0;i<nsolns;i++) {
       std::cout<<(q6dof_solns[i]).transpose();
    }*/
    
    Eigen::Affine3d a_tool_des; // expressed in DH frame
    
    a_tool_des.linear() = R_des;
    // a_tool_des.translation() << x_des,0,0;
    // double x_des;
    // while (true) {
        // std::cout<<std::endl<<"enter desPt_x: ";
        // std::cin>>desPt.x;

        // p[0] = desPt.x;
        // reachPtVec.clear();
        std::cout << "====  irb120 kinematics solver ====" << std::endl;

        // output a File 
        std::ofstream outputFile;
        // std::ios::trunc If the file is opened for output operations and it already existed, its previous content is deleted and replaced by the new one.
        outputFile.open("mktPtPos.txt");
        for (double x_des = -0.4; x_des < 0.7; x_des += 0.1) {
            std::cout << std::endl;
            std::cout << "x=" << x_des <<"  ";
            for (double z_des = 0.9; z_des >- 0.4; z_des -= 0.1) {
                std::cout << std::endl;
                std::cout << "z=" << round(z_des*10) << "  ";
               for (double y_des =- 1.0; y_des < 1.0; y_des += 0.05) {
                    p[0] = x_des;
                    p[1]= y_des;
                    p[2] = z_des;
                    /*desPt.y = y_des;
                    desPt.z = z_des;*/
                    a_tool_des.translation() = p;
                    int nsolns = ik_solver.ik_solve(a_tool_des);
                    //std_msgs::Int16 iknsolns; // create a variable of type "Int16" to publish the number of IK solution
                    //iknsolns.data = nsolns;
                    std::cout<<nsolns;

                    // Only when nsolns > 0, it is meaningful to publish the corresponding desired tool point as a topic: reachablePt
                    if (nsolns>0) {
                        desPt.x = x_des; // remember the desired value of x coordiate and and assign to desPt.x
                        desPt.y = y_des; // remember the desired value of y coordiate and and assign to desPt.y
                        desPt.z = z_des; // remember the desired value of z coordiate and and assign to desPt.z
                        //ROS_INFO("desired point: x = %f, y = %f, z = %f", desPt.x, desPt.y, desPt.z);
                        ik_solver.get_solns(q6dof_solns);
                        // defining a joint limits vector for joint 0 and joint 1, such that each joint is specified within a range of motion
                        std::vector<double> jointLimits {0,-M_PI,-M_PI/2,M_PI/6}; 
                        std::vector<int> weight{1,2,3,3,2,1}; //defining a weight vector
                        double sum;
                        double minimum = 1e6;
                        std_msgs::Int16 bestIkSoluNo;
                        Vectorq6x1 oneIkSolu;
                        // Once nsolns > 0, choose a IK solution both meet the specified joint limit the weight requirement
                        for (int i = 0; i < q6dof_solns.size(); ++i){
                            oneIkSolu = q6dof_solns[i];
                            if (oneIkSolu[0] < jointLimits[0] && oneIkSolu[0] > jointLimits[1] 
                                && oneIkSolu[1] < jointLimits[3] && oneIkSolu[1] > jointLimits[2]) {
                                sum = 0;
                                for (int ijnt = 0; ijnt < 6; ++ijnt){
                                    sum = sum + oneIkSolu[ijnt] * weight[ijnt];
                                }
                                if (sum < minimum){
                                    minimum = sum;
                                    bestIkSoluNo.data = i; // remember the IK solution which has the minimum last joint angle solution
                                }
                            }   
                        }
                        // bestIkSoluNumPub.publish(bestIkSoluNo); // publish the value--of type stf_msgs::Int16-- to the topic "ikSoluNum"
                        // reachPtPub.publish(desPt); // publish the value--of type geometry_msgs::Point-- to the topic "reachablePt"
                        outputFile << desPt << std::endl;
                    }
                }
            }
        }
        // for (int i = 0; i < reachPtVec.size(); ++i) {
        //     ROS_INFO("Reachable Point Coordinates: X = %f, Y= %f, Z = %f", reachPtVec[&i].x ,reachPtVec[&i].y, reachPtVec[&i].z);
        // }
        outputFile.close();
        // timer.sleep();
    // }
    return 0;
}

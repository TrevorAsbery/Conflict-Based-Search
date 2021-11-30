#include "CBS.h"
#include <iostream>
#include <queue>

vector<Path> CBS::find_solution() {
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list

    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);  // whenever generating a new node, we need to
                                 // put it into all_nodes
                                 // so that we can release the memory properly later in ~CBS()

    // find paths for the root node
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) {
        root->paths[i] = a_star.find_path(i);
        if (root->paths[i].empty()) {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }

    //compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // put the root node into open list
    open.push(root);

    while (!open.empty()) {
        //get P node to prune
        CBSNode* P = open.top();

        //pop current node from the queue
        open.pop();

        //make a list
        list<Constraint> collisions = find_collisions(P->paths);
        
        //if there are no newly found constraints, there are no collisions
        //return paths
        if(!collisions.size()){
            return P->paths;
        }

        //loop through all the collisions
        for(auto c: collisions){

            //create new CBS node
            CBSNode* Q = new CBSNode(*P);

            //add the constraint to the parent nodes list of constraints
            Q->constraints = P->constraints;
            Q->constraints.push_back(c);

            //set the paths equal
            Q->paths = P->paths;

            //get the agent for that particular constraint
            int agent = get<0>(c);

            //find a new path for the agent with that constraint
            Path new_path = a_star.find_path(agent, Q->constraints);

            //if the newly created path exists
            if(new_path.size()){

                //set the new path of the agent in the node
                Q->paths[agent] = new_path;

                //update the costs
                for (int i=0; i<Q->paths.size(); i++){
                    Q->cost += (int)Q->paths[i].size() - 1;
                }
                
                //push this node int the queue
                open.push(Q);
            }


        }

    }

    return vector<Path>(); // return "No solution"
}


//function returns constraints of any collisions on the path
list<Constraint> CBS::find_collisions(vector<Path> paths){

    //for every path
    for(int i=0; i<paths.size(); i++){
        //Check that path against every other path
        for(int j=i+1; j<paths.size(); j++){

            //find the smaller and larger of the two agent paths
            int smaller_pathsize = 0;
            int larger_pathsize = 0;
            int larger_path = -1;
            int smaller_path = -1;
            
            if(paths[i].size()<paths[j].size()){
                 smaller_pathsize = paths[i].size();
                 larger_pathsize = paths[j].size();
                 larger_path = j;
                 smaller_path = i;
            }
            else{
                smaller_pathsize = paths[j].size();
                larger_pathsize = paths[i].size();
                larger_path= i;
                smaller_path = j;
            }
            
            //iterate through the smaller path and check for collisions
            for(int t=1; t<smaller_pathsize; t++){

                //if two paths collide on the same edge return two edge constraints
                //one for each agent
                if(paths[i][t]==paths[j][t-1] && paths[i][t-1]==paths[j][t]){
                    list<Constraint> collisions;
                    Constraint Ce1 = {i, paths[i][t-1], paths[i][t], t, EDGE};
                    collisions.push_back(Ce1);
                    Constraint Ce2 = {j, paths[j][t-1], paths[j][t], t, EDGE};
                    collisions.push_back(Ce2);
                    return collisions;
                
                }

                //if two paths collide on the same vertex return two vertex constraints
                //one for each agent
                if(paths[i][t]==paths[j][t]){
                    list<Constraint> collisions;
                    Constraint Cv1 = {i, paths[i][t], -1, t, VERTEX};
                    collisions.push_back(Cv1);
                    Constraint Cv2 = {j, paths[j][t], -1, t, VERTEX};
                    collisions.push_back(Cv2);
                    return collisions;
                
                }
            }

            //if the larger path collides with the smaller path's edge locations
            //return a goal constraint
            for(int t=smaller_pathsize; t<larger_pathsize; t++){
                if(a_star.ins.goal_locations[smaller_path]==paths[larger_path][t]){
                    list<Constraint> collisions;
                    Constraint Cg1 = {larger_path, a_star.ins.goal_locations[smaller_path], -1, smaller_pathsize-1, GOAL};
                    collisions.push_back(Cg1);
                    return collisions;
                }
            }

        }
    }

    //if nothing else is returned, return an empty list of constraints
    return list<Constraint>();

}


CBS::~CBS() {
    // release the memory
    for (auto n : all_nodes)
        delete n;
}

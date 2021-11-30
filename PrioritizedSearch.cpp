#include <PrioritizedSearch.h>

vector<Path> PrioritizedSearch::find_solution() {

  int num_of_agents = a_star.ins.num_of_agents;
    vector<Path> paths(num_of_agents);

    // assign priority ordering to agents
    // By default, we use the index ordering of the agents where
    // the first always has the highest priority.
    list<int> priorities;
    for (int i = 0; i < num_of_agents; i++) {
        priorities.push_back(i);
    }

    // plan paths
    for (int i : priorities) {

      list<Constraint> constraints;

      //build the constraints based on the previously created paths
      //for all previously built plans
      for(int p=0; p<i; p++){
        //go to each planning location and build a constraint that the current path shall not cross
        for(int j=1; j<paths[p].size(); j++){

            //vertex constraint
            //two paths aren't at the same vertex at the same time
            Constraint Cv1 = {i, paths[p][j], -1, j, VERTEX};
            constraints.push_back(Cv1);
            
            //edge constraint
            //two paths never traverse the same edge in the opposite direction
            Constraint Ce2 = {i, paths[p][j], paths[p][j-1], j, EDGE};
            constraints.push_back(Ce2);

            //if a path crosses a goal state, make sure the agent 
            //can't go to that goal state at any time before or at when the path crossses it
            if(paths[p][j]==a_star.ins.goal_locations[i]){
              for(int a=0; a<j+1; a++){
                Constraint Cv2 = {i, paths[p][j], -1, a, VERTEX};
                constraints.push_back(Cv2);
                  }
              }

            //if a path gets to its goal state, make sure that location can never be crossed again
             if(paths[p][j]==a_star.ins.goal_locations[p]){

               Constraint Cg1 = {i, paths[p][j], -1, j, GOAL};
                constraints.push_back(Cg1);
              }
          }
        }

              //once all constraints are made for the agent, find the path
              paths[i] = a_star.find_path(i, constraints);

              if (paths[i].empty()) {
                paths.resize(i);
                return paths;
                }
  }
    return paths;
}



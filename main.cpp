#include "Agent.h"
#include <fstream>
#include <iostream>
using namespace std;

void InitializeAgent(AgentController*& _agents);


int main(int _argc, char** _argv){
  srand(static_cast<unsigned int>(time(NULL)));

  char next = 'Y';
  AgentController* agent = NULL;
  InitializeAgent(agent);

  //main render loop
  while(true){
    //update AgentStates
    while(agent->Update());
    cout << "Continue? (Y/N)" << endl;
    cin >> next;
    if (next != 'Y' && next != 'y')
      break;
    agent->SetGoal(RandomQuery());
  }

  delete agent;
  agent  = NULL;

  return 0;
}//end main

void InitializeAgent(AgentController*& _agent){
  _agent = new AgentController;
  _agent->SetState(RandomQuery());
  _agent->SetGoal(RandomQuery());
}


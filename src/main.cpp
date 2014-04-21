#include "Container.h"
#include "Combinator.h"
#include "Discretizer.h"
#include "Types.h"

void testContainer(Container<int>&);
void testDiscretizer(Container<int>&);
void testCombinator(Container<int>&,Container<int>&);

int main(void)
{
  Container<int> state1;
  state1.push_back(1);
  state1.push_back(2);
  testContainer(state1);
  testDiscretizer(state1);
  testCombinator(state1,state1);
  std::cout << "Hello world!";
  return 0;
}

void testContainer(Container<int>& state){
  state.print();
}

void testDiscretizer(Container<int>& state){
  Discretizer<int> discretizer(state);
  std::cout << discretizer() << std::endl;
}

void testCombinator(Container<int>& state1, Container<int>& state2){
  Combinator<int> combinator(state1);
  std::cout << combinator(state2) << std::endl;
}

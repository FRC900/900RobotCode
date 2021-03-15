#include <iostream>
#include <cctype>
#include <string>
#include <vector>

struct point {
  double metric_x = -1; //return horizontal distance in cm
  double metric_y = -1; //return vertical distance in cm
};
constexpr char min_y = 'A'; //highest row of "actual" points
constexpr char top_row = 'T'; //edge above A
std::vector<std::string> bad_coords = {"A2","A5","A11","E2","E5","E11","C1","C2","C4","C5","C6","C7","C8","C9","C10","C11"}; //Any points that do not exist
bool user_friendly (int x_, char& y_){
  bool answer = true; //return variable, true = pass, false = fail
  constexpr char max_y = 'F'; //lowest edge, one below the "actual" point rows
  constexpr int min_x = 0; //farthest left edge, one before the "actual" point columns
  constexpr int max_x = 12; // farthest right edge, one after the "actual" point columns
  std::string whole_point = y_ + std::to_string(x_); //converts point input to one string for checking against nonexistant points
  if (x_ < min_x || x_ > max_x)
  {
    answer = false;
  }
  else if (y_ == top_row)
  {
    y_ = 64; //sets the y_coord in convert_to_metric to the correct ASCII equivalent, allows for changes to top_row
  }
  else if (y_ < min_y || y_ > max_y)
  {
    answer = false;
  }
  else {
    for (int i = 0; i < bad_coords.size(); i++)
    {
      if (whole_point == bad_coords[i])
      {
        answer = false;
      }
    }
  }
  return answer;
}

bool convert_to_metric(int x_coord, char y_coord, point& input){
  y_coord = toupper(y_coord);
  constexpr int max_y_dist = 381; //height of the highest row in cm, not including the top edge
  constexpr double grid_dist = 76.2; //distance between two rows or columns in cm
  if (user_friendly(x_coord, y_coord)) //converts to metric with valid input
  {
    input.metric_x = (x_coord*grid_dist);
    input.metric_y = (max_y_dist-(y_coord-min_y)*grid_dist);
    std::cout<<input.metric_x<<" cm, "<<input.metric_y<<" cm"<<std::endl;
    return true;
  }
  else //error message on a bad input
  {
    std::cout<<"Invalid input. Please retry with the format: <letter a-f> <number 0-12>"<<std::endl;
    std::cout<<"Nonexistant points:"<<std::endl;
    for(int i = 0; i<bad_coords.size(); i++){
      if (bad_coords.size()-i != 1) {
        std::cout<<bad_coords[i]<<", ";
      }
      else {
        std::cout<<"and "<<bad_coords[i]<<"."<<std::endl;
      }
    }
    return false;
  }
}

int main(int argc, char const *argv[]){
  point point_1;
  char y;
  int x;
  std::cout<<"Input field coordinates"<<std::endl;
  std::cin>>y;
  std::cin>>x;
  convert_to_metric(x, y, point_1);
}

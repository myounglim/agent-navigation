#include "GridPathPlanner.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>


/*
* Initialize member variables like width, height, useAdaptive, and set up the 2d vectors to correct size with the width and height 
* Also initialize the heuristic values initially to the manhattan distance 
*/
GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star) {
	// TODO
	//f = g + h
	mBoardWidth = grid->GetWidth();
	mBoardHeight = grid->GetHeight(); 
	mUseAdaptive = use_adaptive_a_star; 

	mHeuristicMatrix.resize(mBoardHeight, std::vector<int>(mBoardWidth, 0));
	mCostMatrix.resize(mBoardHeight, std::vector<int>(mBoardWidth, 1000));
	mClosed.resize(mBoardHeight, std::vector<int>(mBoardWidth, 0));

	mAction.resize(mBoardHeight, std::vector<int>(mBoardWidth, -1));
	mPolicy.resize(mBoardHeight, std::vector<int>(mBoardWidth, -1));

	xyLoc goal = grid->GetGoalLocation(); 
	xyLoc current = grid->GetCurrentLocation(); 
	std::cout<<"Agent is at X:" << current.x << " Y: " << current.y << "\n"; 
	std::cout<<"Goal is at X:" << goal.x << " Y: " << goal.y << "\n\n"; 
	
	//initialize the heuristic values according to Manhattan Distance 
	for(int rows = 0; rows < mBoardHeight; ++rows) {
		for(int cols = 0; cols < mBoardWidth; ++cols) { 
			mHeuristicMatrix[rows][cols] = std::abs(cols - goal.x) + std::abs(rows -goal.y);
		}
	}

}

GridPathPlanner::~GridPathPlanner(){
	// TODO
	std::vector<OpenMatrix>().swap(mOpen);
	std::vector< std::vector<int> >().swap(mHeuristicMatrix);
	std::vector< std::vector<int> >().swap(mCostMatrix);
	std::vector< std::vector<int> >().swap(mClosed);
	std::vector< std::vector<int> >().swap(mAction);
	std::vector< std::vector<int> >().swap(mPolicy);
}

/*
* Run A* algorithm, then build the path that the agent should follow to reach the goal, and return the first step that the agent should take according to the built path
* Running A* takes place primarily in the ExpandList function but prior, need to first sort the list. 
* After A* finishes(which is when the goal is reached) the function BuildPath lays out the actions the agent should take to reach the goal.
* If the bool useadaptive is set, then update the heuristics according to h(s) = g(goal) - g(s)
*/
xyLoc GridPathPlanner::GetNextMove(PartiallyKnownGrid* grid) {
	// TODO
	xyLoc goal = grid->GetGoalLocation(); 
	xyLoc current = grid->GetCurrentLocation(); 
	if(mUseAdaptive) {
		UpdateHeuristics(goal); 
	}
	Reset(); 
	mClosed[current.y][current.x] = 1; 
	mCostMatrix[current.y][current.x] = 0; 
	int g = 0;
	int x = current.x;
	int y = current.y;
	int h = mHeuristicMatrix[y][x];
	int f = g + h;
	mOpen.push_back(OpenMatrix(f, g, x, y));

	//f = g + h
	bool found = false; 
	while(!found)
	{
		if(mOpen.empty())
		{
			std::cout<<"mOpen list is empty, could not find a path returning..." << "\n";
			return kInvalidXYLoc; 
		}
		else
		{
			SortList(); 
			found = ExpandList(grid);
		}
	}		//end while loop
	/*
	std::cout<<"Goal g_value: " << mCostMatrix[goal.y][goal.x] << "\n";
	for(int i = 0; i < mClosed.size(); i++) {
		for(int j = 0; j < mClosed[i].size(); j++)
		{
			std::cout << mClosed[i][j] << " ";
		}
		std::cout<<"\n";
	}
	*/
	BuildPath(current, goal);
	numExpansions.push_back(GetNumExpansions());
	current = grid->GetCurrentLocation(); 
	x = current.x + DELTA[mPolicy[current.y][current.x]][1];
	y = current.y + DELTA[mPolicy[current.y][current.x]][0];

	return xyLoc(x,y);
}

/*
* Count the number of expansions after each search 
*/
int GridPathPlanner::GetNumExpansions() {
	// TODO
	int numCounter = 0; 
	for(int i = 0; i < mClosed.size(); ++i)
	{
		for(int j = 0; j < mClosed[i].size(); ++j)
		{
			if(mClosed[i][j] == 1)
				numCounter++; 
		}
	}
	return numCounter; 
}

/*
* Sort the list first by f-value. Then if ties occur, sort according to greater g-value(within the indices of the lowest f-value) 
* If the g-values are still tied, then break ties by lower xyLoc
*/
void GridPathPlanner::SortList() {
	//sorting 
	std::sort(mOpen.begin(), mOpen.end(), sort_by_fvalue()); 
	if(mOpen.size() > 1 && mOpen[0].f_value == mOpen[1].f_value) 
	{
		int lowest_f = mOpen.front().f_value; 
		int stoppingIndex = 0; 
		for (int i = 1; i < mOpen.size(); i++)
		{
			if(mOpen[i].f_value == lowest_f)
				stoppingIndex++; 
			else
				break; 
		}

			std::sort(mOpen.begin(), mOpen.begin() + stoppingIndex + 1, sort_by_gvalue()); 	

			if(stoppingIndex > 0)
			{
				int largest_g = mOpen.front().g_value;
				int stopIndex = 0;
				for(int i = 1; i < mOpen.size(); i++)
				{
					if(mOpen[i].g_value == largest_g)
						stopIndex++; 
					else
						break; 
				}
				
				if(stopIndex > stoppingIndex)
					stopIndex = stoppingIndex;

				std::sort(mOpen.begin(), mOpen.begin() + stopIndex + 1, sort_by_xyloc());
			} 	
	}
}

/*
* After sorting, get the first element from the list and expand up,left,right, and down. Check whether they are valid locations and not already in the 
* closed list, if those conditions are met, push the new location into the open list. Function returns false is goal is reached and true if goal is reached
*/
bool GridPathPlanner::ExpandList(PartiallyKnownGrid* grid) {
	xyLoc goal = grid->GetGoalLocation(); 
	OpenMatrix next = mOpen.front();
	std::swap(mOpen.front(), mOpen.back());
	mOpen.pop_back(); 		

	//reached goal?
	if(next.x_pos == goal.x && next.y_pos == goal.y)
	{
		return true; 
	}
	else
	{
		for(int i = 0; i < 4; i++)
		{
			int x2 = next.x_pos + DELTA[i][1];
			int y2 = next.y_pos + DELTA[i][0];
			xyLoc neighbor(x2,y2);
			if(grid->IsValidLocation(neighbor) && !grid->IsBlocked(neighbor))
			{
				if(mClosed[y2][x2] == 0) 
				{
					int g2 = next.g_value + 1;
					int h2 = mHeuristicMatrix[y2][x2]; 
					int f2 = g2 + h2; 
					OpenMatrix om(f2, g2, x2, y2);
					mOpen.push_back(om);
					mClosed[y2][x2] = 1;
					mAction[y2][x2] = i;
					mCostMatrix[y2][x2] = g2;   
				}
			}
		}
		return false; 	
	}		
}

/*
* Lays out the actions the agent should take to reach the goal. It works by 
* starting backwards from the goal and reversing the action it took to reach that particular cell.
*/
void GridPathPlanner::BuildPath(xyLoc initial, xyLoc goal) {
	//start from the goal
	int x_curr = goal.x;
	int y_curr = goal.y;
	while(x_curr != initial.x || y_curr != initial.y)
	{
		int x_prev = x_curr - DELTA[mAction[y_curr][x_curr]][1];
		int y_prev = y_curr - DELTA[mAction[y_curr][x_curr]][0];
		mPolicy[y_prev][x_prev] = mAction[y_curr][x_curr];
		x_curr = x_prev;
		y_curr = y_prev;  
	}
}

/*
* Reset the data structures to run the next A* search
*/
void GridPathPlanner::Reset() {
	mOpen.clear();
	for(int i = 0; i < mBoardHeight; i++)
	{
		for(int j = 0; j < mBoardWidth; j++)
		{
			mClosed[i][j] = 0; 
			mAction[i][j] = -1;
			mPolicy[i][j] = -1; 
			mCostMatrix[i][j] = 1000; 
		}
	}
}

/*
* Update heuristics if using Adaptive A*
* h(s) = g(goal) - g(s)
*/
void GridPathPlanner::UpdateHeuristics(xyLoc goal) {
	//update the heuristic values 
	int goal_g_value = mCostMatrix[goal.y][goal.x];
	for(int rows = 0; rows < mBoardHeight; ++rows) {
		for(int cols = 0; cols < mBoardWidth; ++cols) {
			if(mPolicy[rows][cols] != -1) {
				mHeuristicMatrix[rows][cols] = goal_g_value - mCostMatrix[rows][cols];
			}
		}
	}
	/*
	std::cout<<"Updating heuristics table...\n";
	for(int rows = 0; rows < mBoardHeight; ++rows) {
		for(int cols = 0; cols < mBoardWidth; ++cols) {
			std::cout << std::setfill('0') << std::setw(2);
			std::cout<< mHeuristicMatrix[rows][cols] << "  ";
		}
		std::cout<<"\n";
	}
	*/
}

/*
* Print all the expansions that took place at each step as well as the total sum and average
*/
void GridPathPlanner::PrintExpansions() {
	std::cout<<"\nExpansions" << "\n";
	int sum = 0; 
	for(int i = 0; i < numExpansions.size(); i++)
	{
		std::cout<<numExpansions[i] << " ";
		sum += numExpansions[i];
	}
	std::cout<<"\nSum is: " << sum << "\n";
	float average = static_cast<float>(sum) / numExpansions.size();  
	std::cout<<"Average is: " << average << "\n";
}
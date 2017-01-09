#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H

#include "PartiallyKnownGrid.h"
#include <vector>

struct OpenMatrix {
	OpenMatrix(int fval, int gval, int xpos, int ypos) :f_value(fval), g_value(gval), x_pos(xpos), y_pos(ypos) {}
	int f_value;
	int g_value;
	int x_pos;
	int y_pos;
};

struct sort_by_fvalue {
    inline bool operator() (const OpenMatrix& struct1, const OpenMatrix& struct2)
    {
        return (struct1.f_value < struct2.f_value);
    }
};

struct sort_by_gvalue {
    inline bool operator() (const OpenMatrix& struct1, const OpenMatrix& struct2)
    {
        return (struct1.g_value > struct2.g_value);
    }
};

struct sort_by_xyloc {
    inline bool operator() (const OpenMatrix& struct1, const OpenMatrix& struct2)
    {
       	if (struct1.x_pos == struct2.x_pos)
			return struct1.y_pos < struct2.y_pos;
		else
			return struct1.x_pos < struct2.x_pos;
    }
};


class GridPathPlanner{
public:
	GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star = false);
	~GridPathPlanner();
	
	bool ExpandList(PartiallyKnownGrid* grid); 
	void SortList(); 
	void BuildPath(xyLoc initial, xyLoc goal);
	xyLoc GetNextMove(PartiallyKnownGrid* grid);
	int GetNumExpansions();
	void Reset(); 
	void UpdateHeuristics(xyLoc goal); 
	void PrintExpansions(); 
	const int DELTA[4][2] = {{-1,0}, 	//up
							{0,-1},		//left
							{1,0},		//down
							{0,1}};		//right
private:
	int mBoardWidth;
	int mBoardHeight; 
	bool mUseAdaptive; 
	std::vector< std::vector<int> > mHeuristicMatrix;
	std::vector< std::vector<int> > mCostMatrix;
	std::vector<OpenMatrix> mOpen; 						//{f-value, g-value, x-pos, y-pos}
	std::vector< std::vector<int> > mClosed; 

	std::vector< std::vector<int> > mAction;
	std::vector< std::vector<int> > mPolicy;

	std::vector<int> numExpansions; 
};

#endif

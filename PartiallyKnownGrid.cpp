#include "PartiallyKnownGrid.h"
using namespace std;

PartiallyKnownGrid::PartiallyKnownGrid(std::string instance_name) {
	start_ = kInvalidXYLoc;
	curr_ = kInvalidXYLoc;
	goal_ = kInvalidXYLoc;
	width_ = -1;
	height_ = -1;

	ifstream map;
	map.open(instance_name.c_str());

	// Read the dimensions of the grid.
	string s;
	map >> s;
	if (s == "width")
		map >> width_;
	else
		cout<<"Error reading width!"<<endl;

	map >> s;
	if (s == "height")
		map >> height_;
	else
		cout<<"Error reading height!"<<endl;

	cout<<"Grid dimensions: "<<width_<<" x "<<height_<<endl;

	// Generate the grid.
	grid_.resize(width_);
	for (int x = 0; x < width_; x++)
		grid_[x].resize(height_);

	// Read the cells.
	string grid;
	while (getline(map, s))
		grid = grid + s;

	if (grid.length() != width_ * height_)
		cout<<"Map dimensions do not match the map!"<<endl;

	int i = 0;
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			char cell = grid[i];
			switch (cell) {
			case '#':
				grid_[x][y] = kObstacle;
				break;
			case '.':
				grid_[x][y] = kEmptyCell;
				break;
			case 'H':
				grid_[x][y] = kHiddenObstacle;
				break;
			case 'O':
				grid_[x][y] = kEmptyCell;
				start_ = xyLoc(x,y);
				curr_ = xyLoc(x,y);
				break;
			case '$':
				grid_[x][y] = kEmptyCell;
				goal_ = xyLoc(x,y);
				break;
			default:
				cout<<"Unknown symbol in map: "<<cell<<endl;
				break;
			}

			i++;
		}
	}

	if (start_ == kInvalidXYLoc || goal_ == kInvalidXYLoc)
		cout<<"Start and/or goal not specified!"<<endl;
}

void PartiallyKnownGrid::Reset() {
	curr_ = start_;
	for (int x = 0; x < width_; x++)
		for (int y = 0; y < height_; y++)
			if (grid_[x][y] == kDiscoveredHiddenObstacle)
				grid_[x][y] = kHiddenObstacle;
}

bool PartiallyKnownGrid::MoveTo(xyLoc l) {
	if (!IsValidLocation(l)) {
		cout<<"MoveTo called with a location "<<l<<" that is outside the boundaries of the map!"<<endl;
		return false;
	}

	if (IsBlocked(l)) {
		cout<<"MoveTo called with a location "<<l<<" that contains an obstacle!"<<endl;
		return false;
	}

	int dx = abs(curr_.x - l.x);
	int dy = abs(curr_.y - l.y);

	if (dx + dy != 1) {
		cout<<"MoveTo called with a location "<<l<<" that is not a neighbor of the current cell "<<curr_<<"!"<<endl;
	}

	// Move the agent.
	curr_ = l;

	// Check the surrounding cells for hidden obstacles.
	int x_min = max(l.x-1, 0);
	int x_max = min(l.x+1, width_-1);

	int y_min = max(l.y-1, 0);
	int y_max = min(l.y+1, height_-1);

	for (int x = x_min; x <= x_max; x++)
		for (int y = y_min; y <= y_max; y++)
			if (grid_[x][y] == kHiddenObstacle)
				grid_[x][y] = kDiscoveredHiddenObstacle;

	return true;
}

bool PartiallyKnownGrid::IsValidLocation(xyLoc l) const {
	return l.x >= 0 && l.x < width_ && l.y >= 0 && l.y < height_;
}


bool PartiallyKnownGrid::IsBlocked(xyLoc l) const {
	if (!IsValidLocation(l)) {
		cout<<"IsBlocked called with a location that is out of bounds!"<<endl;
		return false;
	}

	return grid_[l.x][l.y] == kObstacle || grid_[l.x][l.y] == kDiscoveredHiddenObstacle;
}

void PartiallyKnownGrid::DrawGrid(bool clear_terminal) const {
	if (clear_terminal) {
#if defined(_WIN32) || defined(_WIN64)
        system("cls");
#else
        system("clear");
#endif
	}

	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			cellType type = (cellType)grid_[x][y];
			if (type == kEmptyCell) {
				xyLoc l(x,y);
				if (l == curr_)
					cout<<"O";
				else if (l == goal_)
					cout<<"$";
				else
					cout<<".";
			}
			else if (type == kObstacle || type == kDiscoveredHiddenObstacle)
				cout<<"#";
			else if (type == kHiddenObstacle)
				cout<<"H";
		}
		cout<<endl;
	}
}


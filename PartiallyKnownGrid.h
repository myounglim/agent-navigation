#ifndef PARTIALLY_KNOWN_GRID_H
#define PARTIALLY_KNOWN_GRID_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <stdlib.h>

enum cellType {kEmptyCell, kObstacle, kHiddenObstacle, kDiscoveredHiddenObstacle};

struct xyLoc {
public:
	xyLoc() { x = -1; y = -1; }
	xyLoc(int _x, int _y) :x(_x), y(_y) {}
	int x;
	int y;
};

const xyLoc kInvalidXYLoc(-1, -1);

static std::ostream& operator <<(std::ostream & out, const xyLoc &loc) {
	out << "(" << loc.x << ", " << loc.y << ")";
	return out;
}

static bool operator==(const xyLoc &l1, const xyLoc &l2) {
	return (l1.x == l2.x) && (l1.y == l2.y);
}

static bool operator!=(const xyLoc &l1, const xyLoc &l2) {
	return (l1.x != l2.x) || (l1.y != l2.y);
}

static bool operator<(const xyLoc &l1, const xyLoc &l2) {
	if (l1.x == l2.x)
		return l1.y < l2.y;
	else
		return l1.x < l2.x;
}

class PartiallyKnownGrid {
public:
	PartiallyKnownGrid(std::string instance_name = "");
	~PartiallyKnownGrid() {}

	// Sets the current location to the start location and hides all hidden obstacles.
	void Reset();

	// Moves the agent from its current location ('curr_') to the specified location ('l').
	// 'l' needs to be a valid neighbor of 'curr_'.
	// All adjacent hidden obstacles to 'l' are revealed (and will appear blocked when 'IsBlocked' is called.
	// Returns true iff the move is valid.
	bool MoveTo(xyLoc l);

	// Returns true if 'l' is an initially known obstacle, or a discovered hidden obstacle.
	// Returns false if 'l' is not an obstacle or a hidden obstacle that has not been discovered.
	bool IsBlocked(xyLoc l) const;

	int GetWidth() const {return width_;}
	int GetHeight() const {return height_;}

	xyLoc GetCurrentLocation() const {return curr_;}
	xyLoc GetGoalLocation() const {return goal_;}

	bool GoalReached() const {return curr_ == goal_;}
	bool IsValidLocation(xyLoc l) const;

	void DrawGrid(bool clear_terminal = true) const;
private:
	std::vector<std::vector<char> > grid_;
	int width_, height_;
	xyLoc start_, curr_, goal_;
};

#endif


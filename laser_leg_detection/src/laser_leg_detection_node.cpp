#include "ros/ros.h"
#include <iostream>

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

/**
 * Parameters for the Leg Detection Algorithm
 */
static float LEG_MIN_DIFF = 0.095f;
static float LEG_MAX_DIFF = 0.17f;

static float MIDDLE_MIN_DIFF = 0.095f;
static float MIDDLE_MAX_DIFF = 0.7f;

static float L_EXT = 0.2f;
static float L_INT = 0.5f;

static float D_MIN_STEP = 0.0f;
static float D_MAX_STEP = 0.7f;

static float EPSILON = 0.05f;

ros::Subscriber laser_sub;
ros::Subscriber amcl_sub;
ros::Publisher leg_pub;

geometry_msgs::PoseWithCovarianceStamped pose;

/**
 * Class that represents a reading in a LaserScan message, this method stores
 * the distance and the angle (calculated from the LaserScan message) for the given reading.
 */
class LaserReading {

	float _distance;
	float _angle;

public:
	LaserReading() {
		_distance = 0;
		_angle = 0;
	} // ?

	LaserReading(float distance, float angle) {
		_distance = distance;
		_angle = angle;
	}

	float getDistance() {
		return _distance;
	}

	float getAngle() {
		return _angle;
	}

	double getX() {
		return (_distance * cos(_angle));
	}

	double getY() {
		return (_distance * sin(_angle));
	}

	float getDistance(LaserReading other) {
		return (float) sqrt(
				pow(getX() - other.getX(), 2) + pow(getY() - other.getY(), 2));
	}

};

/**
 * Wrapper class for a sequence of LaserReadings, this class contains
 * the length of the sequence, the reading that represents the middle of the
 * sequence and the difference (Euclidean distance) between the first and the last
 * LaserReading in the sequence.
 */
class Sequence {

	LaserReading _sequenceMiddle;
	float _difference;
	int _sequenceLength;

public:
	Sequence(LaserReading reading, float difference, int sequenceLength) {
		_sequenceMiddle = reading;
		_difference = difference;
		_sequenceLength = sequenceLength;
	}

public:
	int getLength() {
		return _sequenceLength;
	}

public:
	LaserReading getMiddle() {
		return _sequenceMiddle;
	}

public:
	float getDifference() {
		return _difference;
	}

};

/**
 * Class that represents a potential leg pattern (a collection of Sequence objects)
 *  this may or may not represent an actual leg and must be filtered before used.
 */
class LegPattern {
	static const int FIRST_LEG_INDEX = 1;
	static const int SECOND_LEG_INDEX = 3;

	std::vector<Sequence> *_pattern;

public:
	LegPattern(std::vector<Sequence> *pattern) {
		_pattern = pattern;
	}

public:
	Sequence getFirstLeg() {
		return _pattern->at(FIRST_LEG_INDEX);
	}

public:
	Sequence getBetweenLegs() {
		return _pattern->at(SECOND_LEG_INDEX - 1);
	}

public:
	Sequence getSecondLeg() {
		return _pattern->at(SECOND_LEG_INDEX);
	}

public:
	std::vector<Sequence> getPattern() {
		return *_pattern;
	}
};

class Point2D {
	double x;
	double y;

public:
	Point2D(double newX, double newY) {
		x = newX;
		y = newY;
	}

	double getX() {
		return x;
	}

	double getY() {
		return y;
	}

	void setX(double newX) {
		x = newX;
	}

	void setY(double newY) {
		y = newY;
	}
};

/**
 *
 * @param q Quaternion describing a particular orientation about the z-axis
 * @return Equivalent orientation about the z-axis in radians
 */

double getHeading(geometry_msgs::Quaternion q) {
	double w = q.w;
	double x = q.x;
	double y = q.y;
	double z = q.z;

	double yaw = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z);
	return yaw;
}

/**
 * Given a candidate pattern, this method will return whether the given pattern is a
 * valid leg pattern or not.
 *
 * @param pattern Pattern to check.
 * @return Whether the given pattern is a leg or not.
 */
int validPattern(LegPattern pattern) {
	float dist = pattern.getFirstLeg().getMiddle().getDistance(
			pattern.getSecondLeg().getMiddle());

	return dist > D_MIN_STEP && dist < D_MAX_STEP
			&& pattern.getFirstLeg().getDifference() > LEG_MIN_DIFF
			&& pattern.getFirstLeg().getDifference() < LEG_MAX_DIFF
			&& pattern.getSecondLeg().getDifference() > LEG_MIN_DIFF
			&& pattern.getSecondLeg().getDifference() < LEG_MAX_DIFF
			&& pattern.getBetweenLegs().getDifference() > MIDDLE_MIN_DIFF
			&& pattern.getBetweenLegs().getDifference() < MIDDLE_MAX_DIFF
			&& pattern.getFirstLeg().getMiddle().getDistance() < 4
			&& pattern.getSecondLeg().getMiddle().getDistance() < 4;
}

/**
 * Given a list of candidate patterns, this method will filter the list
 * and return a list of valid patterns.
 *
 * @param patterns std::vector of candidate patterns.
 * @return A filtered list of patterns which, according to this algorithm,
 * is a valid leg pattern.
 */
std::vector<LegPattern> filterPatterns(std::vector<LegPattern> patterns) {
	std::vector<LegPattern> *filteredPatterns = new std::vector<LegPattern>();
	for (int i = 0; i < patterns.size(); i++) {
		if (validPattern(patterns.at(i))) {
			filteredPatterns->push_back(patterns.at(i));
		}
	}

	return *filteredPatterns;
}

/**
 * Given the current sequence and the current list of candidate sequences that
 * form a partial pattern.
 *
 * @param sequence Sequence that is to be added to the sequence.
 * @param currentSequence Current list of sequences.
 * @return Whether the given sequence currently satisfies the current pattern.
 */
int satisfyPattern(Sequence sequence, std::vector<Sequence> currentSequence) {
	LaserReading value = sequence.getMiddle();

	switch (currentSequence.size()) {

	case 1: {
		return abs(currentSequence.at(0).getMiddle().getX() - value.getX())
				> L_EXT;
	}

	case 2: {
		return abs(value.getX() - currentSequence.at(1).getMiddle().getX())
				> L_INT;
	}

	case 3: {
		return abs(currentSequence.at(2).getMiddle().getX() - value.getX())
				> L_INT;
	}

	case 4: {
		return abs(value.getX() - currentSequence.at(3).getMiddle().getX())
				> L_EXT;
	}

	}
	return false;
}

/**
 * Given an initial index, a list of LaserReadings and a 'jump amount'.
 * This function returns the length of the sequence such that each reading in the sequence
 * is within 'jump amount' of distance from the previous reading.
 *
 * @param h Start index.
 * @param data std::vector of laser readings.
 * @param epsilon 'Jump amount'
 * @return The length of the sequence.
 */
int getSequenceLength(int h, std::vector<LaserReading> data, float epsilon) {
	int k = 1;
	for (int i = h + 1; i < data.size(); i++) {
		LaserReading current = data.at(i);
		LaserReading previous = data.at(i - 1);

		float diff = previous.getDistance(current);

		if (diff < epsilon)
			k++;
		else
			break;
	}

	return k;
}

/**
 * Return whether, according to the other sequence, the sequence d is a min or max sequence.
 *
 * @param sequence Sequence to check against.
 * @param d Sequence to get min/max for.
 * @return Whether the given sequence, d, is a min or max.
 */
int getMinMax(Sequence sequence, Sequence d) {
	if (sequence.getMiddle().getDistance() > d.getMiddle().getDistance())
		return -1;
	else if (sequence.getMiddle().getDistance() < d.getMiddle().getDistance())
		return 1;
	else
		return 0;
}

/**
 * Given a LaserScan message, this method will return a list of leg patterns
 * found in this data.
 *
 * @param laserMessage LaserScan message.
 * @return std::vector of leg patterns in the LaserScan message.
 */
std::vector<LegPattern> getPeoplePose(sensor_msgs::LaserScan laserMessage) {
	std::vector<LaserReading> *readings = new std::vector<LaserReading>();

	std::vector<float> ranges = laserMessage.ranges;

	//Create a list of readings from the LaserScan message.
	float currentAngle = laserMessage.angle_min;
	for (int z = 0; z < ranges.size(); z++) {
		float reading = ranges[z];
		if (reading > laserMessage.range_min
				&& reading < laserMessage.range_max) {
			LaserReading *nlr = new LaserReading(reading, currentAngle);
			readings->push_back(*nlr);
		} else if (reading == 0) {
			LaserReading *nlr = new LaserReading(4, currentAngle);
			readings->push_back(*nlr);
		}

		currentAngle += laserMessage.angle_increment;
	}

	//Create a list of sequences from the calculated list of laser readings.
	std::vector<Sequence> *sequenceMiddles = new std::vector<Sequence>();
	for (int i = 0; i < readings->size(); i++) {
		int sequenceLength = getSequenceLength(i, *readings, EPSILON);

		Sequence *sequence = new Sequence(
				readings->at(i + (sequenceLength / 2)),
				readings->at(i).getDistance(
						readings->at(i + sequenceLength - 1)), sequenceLength);
		sequenceMiddles->push_back(*sequence);

		i += sequenceLength;
	}

	std::vector<LegPattern> *patterns = new std::vector<LegPattern>();

	//Submit the first sequence as the 'base'
	std::vector<Sequence> *s = new std::vector<Sequence>();
	s->push_back(sequenceMiddles->at(0));

	for (int i = 1; i < sequenceMiddles->size(); i++) {
		//Get the next sequence
		Sequence d = sequenceMiddles->at(i);

		int val = getMinMax(sequenceMiddles->at(i - 1), d);
		if (val == 1 || val == -1) {
			//Check whether the current sequence still satisfies the pattern when added to it
			if (satisfyPattern(d, *s)) {
				s->push_back(d);

				//We have a valid pattern, add it to pattern list
				if (s->size() == 5) {
					LegPattern *nlp = new LegPattern(s);
					patterns->push_back(*nlp);
				}
				//We have an invalid pattern, reset
			} else if (val == 1) {
				s = new std::vector<Sequence>();
				s->push_back(d);
			}
		}
	}

	//Return the filtered patterns
	return filterPatterns(*patterns);
}

void amcl_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl) {
//TODO synchronized (LegDetectionNode.this)
	pose = *amcl;
}

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& laserMessage) {
	std::vector<LegPattern> patterns = getPeoplePose(*laserMessage);

	//For each pattern, calculate the position in the map frame and publish it
	for (int i = 0; i < patterns.size(); i++) {
		std::cout << "Leg detected.\n";
		if (&pose != NULL) {
			geometry_msgs::PointStamped originalLeg;

			geometry_msgs::Point *legPoint = &originalLeg.point;
			LaserReading firstLeg = patterns.at(i).getFirstLeg().getMiddle();

			geometry_msgs::Pose currentPose = pose.pose.pose;

//			AffineTransform transform = new AffineTransform();
//			transform.translate(currentPose.position.y,
//					currentPose.position.x);
//			transform.rotate(-getHeading(currentPose.orientation));
//
//			Point2D pt = new Point2D.Double();
//			pt.setLocation(firstLeg.getY(), firstLeg.getX());
//
//			Point2D newPt = new Point2D.Double();
//			transform.transform(pt, newPt);

			Point2D *newPt = new Point2D(firstLeg.getX(), firstLeg.getY());

			legPoint->x = newPt->getX();
			legPoint->y = newPt->getY();
			legPoint->z = 0;

			std::cout << "x " << legPoint->x << " y " << legPoint->y << std::endl;

			std_msgs::Header *originalHeader = &originalLeg.header;
			originalHeader->frame_id = "/base_link";
			leg_pub.publish(originalLeg);
		}

	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "leg_detection");

	ros::NodeHandle n;

	leg_pub = n.advertise<geometry_msgs::PointStamped>("legs", 1000);

	amcl_sub = n.subscribe("amcl_pose", 1, amcl_cb);

	laser_sub = n.subscribe("scan", 1, laser_cb);

	while (ros::ok()) {
		ros::spinOnce();
	}

	return 0;
}

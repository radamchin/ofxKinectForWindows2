#pragma once

#include "ofMain.h"

#include "BaseImage.h"
#include "../Utils.h"

#include "../Data/Body.h"
#include "../Data/Joint.h"

#include <Kinect.VisualGestureBuilder.h>

namespace ofxKinectForWindows2 {
	namespace Source {

		struct GestureState {
			int body;					// index of body associated too
			BOOLEAN continuous;			// Is it a continuous gesture (if false, discrete)
			float value;				// Confidence or Progress
			BOOLEAN detected;			// Mainly for discrete gesture.
			BOOLEAN firstFrameDetected; // discrete gestures
			int id;						// index of the gesture
			uint64_t update_time;		// ofGetElapsedTimeMillis() of when update occured
			string name;
		};

		// -------
		class Body : public BaseFrame<IBodyFrameReader, IBodyFrame> {

		public:
			string getTypeName() const override;
			void init(IKinectSensor *, bool) override;
			bool initGestures(IKinectSensor *, wstring db_file);

			void update(IBodyFrame *) override;
			void update(IMultiSourceFrame *) override;

			void drawProjected(int x, int y, int width, int height, ProjectionCoordinates proj = ColorCamera);
			void drawWorld( ofColor col = ofColor::black );

			ICoordinateMapper * getCoordinateMapper();

			const vector<Data::Body> & getBodies() const;
			const Data::Body & getBody(int n = 0) { return bodies[n]; }
			map<JointType, ofVec2f> getProjectedJoints(int bodyIdx, ProjectionCoordinates proj = ColorCamera);

			const Vector4 getFloorClipPlane() {
				return floorClipPlane;
			}

			ofMatrix4x4 getFloorTransform();

			static void drawProjectedBone(map<JointType, Data::Joint> & pJoints, map<JointType, ofVec2f> & pJointPoints, JointType joint0, JointType joint1, ofColor color = ofColor::green);
			static void drawProjectedHand(HandState handState, ofVec2f & handPos);

			// Gestures

			void setUseGestureDetectionZone(bool state) { 
				ofLogNotice("setUseGestureDetectionZone") << state;
				useGesturesDetectionZone = state;
			}

			void setGestureDetectionZoneBounds(float min_x, float max_x, float min_z, float max_z) {
				ofLogNotice("setGestureDetectionZoneBounds") << "x="<< min_x << ".." << max_x << ", z=" << min_z << ".." << max_z;
				gz_min_x = min_x;
				gz_max_x = max_x;
				gz_min_z = min_z;
				gz_max_z = max_z;
			}

			bool getGestureReaderPausedState(int body_index);
			void setGestureReaderPausedState(int body_index, bool state); 

			void processGestures(vector<int> &tracked_body_ids);
			
			const bool &getGestureIsContinuous(int body_index, int n) { return gesture_states[body_index][n].continuous; }
			const bool &getGestureIsFirstFrameDetected(int body_index, int n) { return gesture_states[body_index][n].firstFrameDetected; }
			const bool &getGestureDetected(int body_index, int n) { return gesture_states[body_index][n].detected; }
			const float &getGestureValue(int body_index, int n) { return gesture_states[body_index][n].value; } // progress or confidence.
			const string &getGestureName(int body_index, int n) { return gesture_states[body_index][n].name; }
			const int &getGestureID(int body_index, int n) { return getGestureDetected(body_index, n) ? gesture_states[body_index][n].id : -1; } // internal tracking/body id it is linked ot.
			int getGestureCount() { return pGesture.size(); }

			const ofColor getColor(int body_index) { return colors[body_index]; }

		protected:
			void initReader(IKinectSensor *) override;

			ICoordinateMapper * coordinateMapper;

			Vector4 floorClipPlane;

			vector<Data::Body> bodies;

			uint64_t gesture_last_unpause_times[BODY_COUNT];// 
			vector< vector<GestureState> > gesture_states;
			IVisualGestureBuilderDatabase * database;
			vector<IGesture *> pGesture;
			IVisualGestureBuilderFrameSource* pGestureSource[BODY_COUNT];
			IVisualGestureBuilderFrameReader* pGestureReader[BODY_COUNT];
			bool useGestures;

			bool useGesturesDetectionZone;
			float gz_min_x;
			float gz_max_x;
			float gz_min_z;
			float gz_max_z;


			vector<ofColor> colors{ ofColor::red, ofColor::green, ofColor::blue, ofColor::magenta, ofColor::cyan, ofColor::orange };
			//ofColor::fromHsb(255 / 0, 200, 255)

		};
	}
}
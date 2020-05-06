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
			int body;				// index of body associated too
			bool continuous;		// Is it a continuous gesture (if false, discrete)
			float value;			// Confidence or Progress
			bool detected;			// Mainly for discrete gesture.
			int id;
			uint64_t update_time;	 // ofGetElapsedTimeMillis() of when update occured
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
			void drawWorld();

			ICoordinateMapper * getCoordinateMapper();

			const vector<Data::Body> & getBodies() const;
			map<JointType, ofVec2f> getProjectedJoints(int bodyIdx, ProjectionCoordinates proj = ColorCamera);

			const Vector4 getFloorClipPlane() {
				return floorClipPlane;
			}

			ofMatrix4x4 getFloorTransform();

			static void drawProjectedBone(map<JointType, Data::Joint> & pJoints, map<JointType, ofVec2f> & pJointPoints, JointType joint0, JointType joint1);
			static void drawProjectedHand(HandState handState, ofVec2f & handPos);

			const bool &getGestureIsContinuous(int body_index, int n) { return gesture_states[body_index][n].continuous; }
			const bool &getGestureDetected(int body_index, int n) { return gesture_states[body_index][n].detected; }
			const float &getGestureValue(int body_index, int n) { return gesture_states[body_index][n].value; } // progress or confidence.
			const string &getGestureName(int body_index, int n) { return gesture_states[body_index][n].name; }
			const int &getGestureID(int body_index, int n) { return getGestureDetected(body_index, n) ? gesture_states[body_index][n].id : -1; } // internal tracking/body id it is linked ot.
			int getGestureCount() { return pGesture.size(); }

		protected:
			void initReader(IKinectSensor *) override;

			ICoordinateMapper * coordinateMapper;

			Vector4 floorClipPlane;

			vector<Data::Body> bodies;
			

			vector< vector<GestureState> > gesture_states;
			IVisualGestureBuilderDatabase * database;
			vector<IGesture *> pGesture;
			IVisualGestureBuilderFrameSource* pGestureSource[BODY_COUNT];
			IVisualGestureBuilderFrameReader* pGestureReader[BODY_COUNT];
			bool useGesture;

		};
	}
}
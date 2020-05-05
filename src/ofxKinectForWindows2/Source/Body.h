#pragma once

#include "ofMain.h"

#include "BaseImage.h"
#include "../Utils.h"

#include "../Data/Body.h"
#include "../Data/Joint.h"

#include <Kinect.VisualGestureBuilder.h>

namespace ofxKinectForWindows2 {
	namespace Source {

		struct GestureResult {
			bool value;
			float progress;
			float confidence;
			int id;
			string name;
		};

		// -------
		class Body : public BaseFrame<IBodyFrameReader, IBodyFrame> {
		public:
			string getTypeName() const override;
			void init(IKinectSensor *, bool) override;
			bool setupVGBF(IKinectSensor *, wstring db_file);

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

			const bool &getGestureResult(int n) { return gestureResults[n].value; }
			const float &getGestureProgress(int n) { return gestureResults[n].progress; }
			const float &getGestureConfidence(int n) { return gestureResults[n].confidence; }
			const string &getGestureName(int n) { return gestureResults[n].name; }
			const int &getGestureID(int n) { return getGestureResult(n) ? gestureResults[n].id : -1; }
			int getGestureSize() { return pGesture.size(); }

		protected:
			void initReader(IKinectSensor *) override;

			ICoordinateMapper * coordinateMapper;

			Vector4 floorClipPlane;

			vector<Data::Body> bodies;
			
			vector<GestureResult> gestureResults;
			IVisualGestureBuilderDatabase * database;
			vector<IGesture *> pGesture;
			IVisualGestureBuilderFrameSource* pGestureSource[BODY_COUNT];
			IVisualGestureBuilderFrameReader* pGestureReader[BODY_COUNT];
			bool useGesture;

		};
	}
}
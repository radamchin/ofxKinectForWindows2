#include "Body.h"
#include "ofMain.h"

namespace ofxKinectForWindows2 {
	namespace Source {
		//----------
		string Body::getTypeName() const {
			return "Body";
		}

		//----------
		const vector<Data::Body> & Body::getBodies() const {
			return bodies;
		}

		//----------
		ofMatrix4x4 Body::getFloorTransform() {
			ofNode helper;
			helper.lookAt(ofVec3f(floorClipPlane.x, floorClipPlane.z, -floorClipPlane.y));
			helper.boom(-floorClipPlane.w);
			ofMatrix4x4 transform = glm::inverse(helper.getGlobalTransformMatrix());
			return transform;
		}

		//----------
		void Body::initReader(IKinectSensor * sensor) {
			this->reader = NULL;
			try {
				IBodyFrameSource * source = NULL;

				if (FAILED(sensor->get_BodyFrameSource(&source))) {
					throw(Exception("Failed to initialise BodyFrame source"));
				}

				if (FAILED(source->OpenReader(&this->reader))) {
					throw(Exception("Failed to initialise BodyFrame reader"));
				}

				SafeRelease(source);
			} catch (std::exception & e) {
				SafeRelease(this->reader);
				throw (e);
			}
		}

		//----------
		void Body::init(IKinectSensor * sensor, bool reader) {
			try {
				BaseFrame::init(sensor, reader);

				if (FAILED(sensor->get_CoordinateMapper(&this->coordinateMapper))) {
					throw(Exception("Failed to acquire coordinate mapper"));
				}

				bodies.resize(BODY_COUNT);

				useGesture = false;

			} catch (std::exception & e) {
				SafeRelease(this->reader);
				throw (e);
			}
		}

		//----------
		bool Body::getGestureReaderPausedState(int body_index) {
			BOOLEAN state = false;

			if (useGesture) {
				if ( FAILED( pGestureReader[body_index]->get_IsPaused(&state) ) ) {
					throw Exception("Failed to get gesture reading paused state");
					return false;
				}
			}

			return state;
		}

		void Body::setGestureReaderPausedState(int body_index, bool state) {

			if (FAILED(pGestureReader[body_index]->put_IsPaused(state))) {
				throw Exception("Failed to set gesture reading paused state");
				return;
			}

			if(!state) gesture_last_unpause_times[body_index] = ofGetFrameNum();

		}

		//----------
		bool Body::initGestures(IKinectSensor * sensor, wstring db_file) {

			if (SUCCEEDED(CreateVisualGestureBuilderDatabaseInstanceFromFile(db_file.c_str(), &database))) {
				
				for (int i = 0; i < BODY_COUNT; i++) {
					if (FAILED(CreateVisualGestureBuilderFrameSource(sensor, 0, &pGestureSource[i]))) {
						throw(Exception("Failed to create VisualGestureBuilderFrameSource"));
						return false;
					}
					if (FAILED(pGestureSource[i]->OpenReader(&pGestureReader[i]))) {
						throw(Exception("Failed to open reader"));
						return false;
					}

					gesture_last_unpause_times[i] = 0; // this will get set to framenum if setGestureReaderPausedState

					setGestureReaderPausedState(i, true);
				}

				UINT gesture_count;
				database->get_AvailableGesturesCount(&gesture_count);
				pGesture.resize(gesture_count);
				database->get_AvailableGestures(gesture_count, &pGesture[0]);

				// Setup the gesture_states with default empty states
				gesture_states.resize(BODY_COUNT);

				for (int b = 0; b < BODY_COUNT; b++) {
					for (int g = 0; g<gesture_count; g++) {
						GestureState state;
						state.id = g;
						state.value = false;
						state.detected = false;
						state.firstFrameDetected = false;
						state.name = "Unknown";
						state.update_time = ofGetElapsedTimeMillis();
						state.body = b;
						gesture_states[b].push_back(state);
					}
				}

				// Read gestures from DB and add to source.
				for (int g = 0; g<gesture_count; g++) {
					GestureType type;
					pGesture[g]->get_GestureType(&type);
					if (pGesture[g] != nullptr) {
						for (int j = 0; j < BODY_COUNT; j++) {
							if (FAILED(pGestureSource[j]->AddGesture(pGesture[g]))) {
								throw(Exception("Failed to add gesture"));
								return false;
							}
							if (FAILED(pGestureSource[j]->SetIsEnabled(pGesture[g], true))) {
								throw(Exception("Failed to setup"));
								return false;
							}

							// Initialise static gesture values from DB.
							const UINT uTextLength = 260;
							wchar_t sName[uTextLength];
							pGesture[g]->get_Name(uTextLength, sName);
							wstring ws(sName);
							string name(ws.begin(), ws.end());

							for (int b = 0; b < BODY_COUNT; b++) {
								gesture_states[b][g].name = name;
								gesture_states[b][g].continuous = (type == GestureType::GestureType_Continuous);
							}

						}
					}
					else {
						throw(Exception("Gesture is null"));
						return false;
					}
				}
				useGesture = true;
				return useGesture;
			}
			return false;
		}

		//----------
		void Body::update(IMultiSourceFrame * multiFrame) {
			this->isFrameNewFlag = false;
			IBodyFrame * frame = NULL;
			IBodyFrameReference * reference;
			try {
				//acquire frame
				if (FAILED(multiFrame->get_BodyFrameReference(&reference))) {
					SafeRelease(reference);
					return; // we often throw here when no new frame is available
				}
				if (FAILED(reference->AcquireFrame(&frame))) {
					SafeRelease(frame);
					return; // we often throw here when no new frame is available
				}
				update(frame);
			} catch (std::exception & e) {
				OFXKINECTFORWINDOWS2_ERROR << e.what();
			}
			SafeRelease(reference);
			SafeRelease(frame);
		}

		//----------
		void Body::update(IBodyFrame * frame) {

			this->isFrameNewFlag = true;
			IFrameDescription * frameDescription = NULL;

			try {

				INT64 nTime = 0;
				if (FAILED(frame->get_RelativeTime(&nTime))) {
					throw Exception("Failed to get relative time");
				}
				
				if (FAILED(frame->get_FloorClipPlane(&floorClipPlane))) {
					throw(Exception("Failed to get floor clip plane"));
				}

				IBody* ppBodies[BODY_COUNT] = {0};
				if (FAILED(frame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies))) {
					throw Exception("Failed to refresh body data");
				}

				int gestures_read_for_body = -1;

				bool canReadGestures = true; // 

				//bool found_valid_body = false;

				for (int i = 0; i < BODY_COUNT; ++i) {
					
					// if (found_valid_body) break; //test already found a body lets see if the gesture bug occurs still

					auto & body = bodies[i];
					body.clear();

					IBody* pBody = ppBodies[i];
					if (pBody) {

						BOOLEAN bTracked = false;
						if (FAILED(pBody->get_IsTracked(&bTracked))) {
							throw Exception("Failed to get tracking status");
						}
						body.tracked = bTracked;
						body.bodyId = i;

						canReadGestures = (gestures_read_for_body == -1) && (ofGetFrameNum() - gesture_last_unpause_times[i] > 1);

						if (bTracked) {

							// found_valid_body = true;

							// Retrieve tracking id
							UINT64 trackingId = 0;

							if (FAILED(pBody->get_TrackingId(&trackingId))) {
								throw Exception("Failed to get tracking id");
							}

							body.trackingId = trackingId;

							if (useGesture) { 
								
								UINT64 gestureTrackingId;
								if(FAILED(pGestureSource[i]->get_TrackingId(&gestureTrackingId))) {
									throw Exception("Failed to get gesture source tracking id");
								}

								if (gestureTrackingId != trackingId) {
									// update the corresponding gesture detector with the new value
									pGestureSource[i]->put_TrackingId(trackingId);

									// From the DiscreteGestureBasics-WPF file.
									// if the current body is tracked, unpause its detector to get VisualGestureBuilderFrameArrived events
									// if the current body is not tracked, pause its detector so we don't waste resources trying to get invalid gesture results
									//  this.gestureDetectorList[i].IsPaused = trackingId == 0; // c# ref. code.

									if (trackingId == 0) {
										setGestureReaderPausedState(i,true);
									} else {
										// Only unpause if valid trackingId and canRead
									//	setGestureReaderPausedState(i, !canReadGestures);
									}

								}

								if (canReadGestures) {
									// unpause
									setGestureReaderPausedState(i, false);
								}

								/*BOOLEAN reading_paused = true;

								if ((FAILED(pGestureReader[i]->get_IsPaused(&reading_paused))))	 {
									throw Exception("Failed to get gesture reading paused state");
								}
								if (reading_paused) {
									pGestureReader[i]->put_IsPaused(false);
									//pGestureSource[i]->put_TrackingId(trackingId);
								}*/
															
							}

							// Retrieve joint position & orientation

							_Joint joints[JointType_Count];
							_JointOrientation jointsOrient[JointType_Count];

							if (FAILED(pBody->GetJoints(JointType_Count, joints))){
								throw Exception("Failed to get joints");
							}
							if (FAILED(pBody->GetJointOrientations(JointType_Count, jointsOrient))){
								throw Exception("Failed to get joints orientation");
							}

							for (int j = 0; j < JointType_Count; ++j) {
								body.joints[joints[j].JointType] = Data::Joint(joints[j], jointsOrient[j], this->getCoordinateMapper());
							}

							// retrieve hand states

							HandState leftHandState = HandState_Unknown;
							HandState rightHandState = HandState_Unknown;

							if (FAILED(pBody->get_HandLeftState(&leftHandState))){
								throw Exception("Failed to get left hand state");
							}
							if (FAILED(pBody->get_HandRightState(&rightHandState))){
								throw Exception("Failed to get right hand state");
							}

							body.leftHandState = leftHandState;
							body.rightHandState = rightHandState;

							if (useGesture){
								if(canReadGestures) {

									gestures_read_for_body = i;

									IVisualGestureBuilderFrame* pGestureFrame = nullptr;

									if (SUCCEEDED(pGestureReader[i]->CalculateAndAcquireLatestFrame(&pGestureFrame))) {
										BOOLEAN bGestureTracked = false;
										if (FAILED(pGestureFrame->get_IsTrackingIdValid(&bGestureTracked))) {
											throw Exception("failed to retrieve tracking id validity");
										}

										// extra sanity check for the multiple body gesture noise errors.
										/*BOOLEAN bFrameSourceActive = false;
										if ( FAILED(pGestureSource[i]->get_IsActive(&bFrameSourceActive)) ) {
											throw Exception("failed to retrieve frame source active");
										}*/

										if (bGestureTracked) { // && bFrameSourceActive

											IDiscreteGestureResult* pGestureResult = nullptr;
											IContinuousGestureResult* pContinuousGestureResult = nullptr;

											for (int g = 0; g < pGesture.size(); g++) {

												// TODO: Could save looking up type everytime by reading gesture_states[i][g].continuous

												GestureType gestureType;
												pGesture[g]->get_GestureType(&gestureType);

												if (gestureType == GestureType::GestureType_Continuous) {

													if (SUCCEEDED(pGestureFrame->get_ContinuousGestureResult(pGesture[g], &pContinuousGestureResult))) {

														float progress;
														pContinuousGestureResult->get_Progress(&progress);
														gesture_states[i][g].detected = true;
														gesture_states[i][g].value = progress;
														gesture_states[i][g].update_time = ofGetElapsedTimeMillis();

														//UINT64 num;
														//pGestureFrame->get_TrackingId(&num);
														//ofLog(OF_LOG_VERBOSE, "gesture:" + ofToString(j) + ", id:" + ofToString(num));
													}
													SafeRelease(pContinuousGestureResult);
												}
												else if (gestureType == GestureType::GestureType_Discrete) {

													if (SUCCEEDED(pGestureFrame->get_DiscreteGestureResult(pGesture[g], &pGestureResult))) {

														// TODO: some extra saftey tests here that this result is connected to the same body/gesture?

														if (pGestureResult != NULL) {

															if (FAILED(pGestureResult->get_Detected(&gesture_states[i][g].detected))) {
																throw Exception("Failed to get discrete gesture detected");
															}

															gesture_states[i][g].update_time = ofGetElapsedTimeMillis();

															if (gesture_states[i][g].detected) {

																if (FAILED(pGestureResult->get_FirstFrameDetected(&gesture_states[i][g].firstFrameDetected))) {
																	throw Exception("Failed to get discrete gesture firstframe detected");
																}

																if (FAILED(pGestureResult->get_Confidence(&gesture_states[i][g].value))) {
																	throw Exception("Failed to get discrete gesture confidence");
																}

																//UINT64 num;
																//pGestureFrame->get_TrackingId(&num);
																//ofLogVerbose() << "body:" << gesture_states[i][g].body << "," << i << ", gesture:" << gesture_states[i][g].id << "," << g << ", name:'" << gesture_states[i][g].name << "', confidence:" + ofToString(confidence,2) + ", tracking-id:"  << num;
															}
														}
													}
												}
											}
											SafeRelease(pGestureResult);
										}
									}
									SafeRelease(pGestureFrame);

								} else {
									// make sure its paused.
									setGestureReaderPausedState(i, true);
								}

							}

						} else { // end if::bTracked

							if (useGesture) {

								/*BOOLEAN reading_paused = false;
								if ((FAILED(pGestureReader[i]->get_IsPaused(&reading_paused)))) {
									throw Exception("Failed to get gesture reading paused state");
								}*/
								//if (!reading_paused) {
									setGestureReaderPausedState(i, true);
									pGestureSource[i]->put_TrackingId(0); // saftey 0 tracking id.
								//}
							}


						}

					}
				}

				for (int i = 0; i < _countof(ppBodies); ++i) {
					SafeRelease(ppBodies[i]);
				}
			} catch (std::exception & e) {
				OFXKINECTFORWINDOWS2_ERROR << e.what();
			}
			SafeRelease(frameDescription);
		}

		//----------
		map<JointType, ofVec2f> Body::getProjectedJoints(int bodyIdx, ProjectionCoordinates proj) {
			map<JointType, ofVec2f> result;

			const auto & body = bodies[bodyIdx];
			if (!body.tracked) return result;

			for (auto & joint : body.joints) {
				ofVec2f & position = result[joint.second.getType()] = ofVec2f();

				TrackingState state = joint.second.getTrackingState();
				if (state == TrackingState_NotTracked) {
					continue;
				}

				position.set(joint.second.getProjected(coordinateMapper, proj));
			}

			return result;
		}

		//----------
		void Body::drawProjected(int x, int y, int width, int height, ProjectionCoordinates proj) {
			ofPushStyle();
			int w, h;
			switch (proj) {
			case ColorCamera: w = 1920; h = 1080; break;
			case DepthCamera: w = 512; h = 424; break;
			}

			const auto & bonesAtlas = Data::Body::getBonesAtlas();

			for (auto & body : bodies) {
				if (!body.tracked) continue;

				map<JointType, ofVec2f> jntsProj;

				ofColor bone_col = getColor(body.bodyId);
				// joint colour? a darker variant of the bone_col?
				ofColor joint_col(bone_col);
				//joint_col.setBrightness(.75); // darker variant

				for (auto & j : body.joints) {
					ofVec2f & p = jntsProj[j.second.getType()] = ofVec2f();

					TrackingState state = j.second.getTrackingState();
					if (state == TrackingState_NotTracked) continue;

					p.set(j.second.getProjected(coordinateMapper, proj));
					p.x = x + p.x / w * width;
					p.y = y + p.y / h * height;

					int radius = (state == TrackingState_Inferred) ? 2 : 8;
					ofSetColor(joint_col);
					ofDrawCircle(p.x, p.y, radius);
				}
				
				for (auto & bone : bonesAtlas) {
					drawProjectedBone(body.joints, jntsProj, bone.first, bone.second, bone_col);
				}

				drawProjectedHand(body.leftHandState, jntsProj[JointType_HandLeft]);
				drawProjectedHand(body.rightHandState, jntsProj[JointType_HandRight]);
			}

			ofPopStyle();
		}

		//----------
		void Body::drawWorld() {
			auto bodies = this->getBodies();
			int bodyIndex = 0;
			for (auto & body : bodies) {
				//draw black lines
				ofPushStyle();
				ofSetLineWidth(10.0f);
				ofSetColor(0);
				body.drawWorld();

				//draw coloured lines
				ofSetLineWidth(8.0f);
				ofColor col(200, 100, 100);
				col.setHue(255.0f / this->getBodies().size() * bodyIndex);
				ofSetColor(col);
				body.drawWorld();

				ofPopStyle();

				bodyIndex++;
			}
		}

		//----------
		ICoordinateMapper * Body::getCoordinateMapper() {
			return this->coordinateMapper;
		}

		//----------
		void Body::drawProjectedBone(map<JointType, Data::Joint> & pJoints, map<JointType, ofVec2f> & pJointPoints, JointType joint0, JointType joint1, ofColor color){
			TrackingState ts1 = pJoints[joint0].getTrackingState();
			TrackingState ts2 = pJoints[joint1].getTrackingState();
			if (ts1 == TrackingState_NotTracked || ts2 == TrackingState_NotTracked) return;
			if (ts1 == TrackingState_Inferred && ts2 == TrackingState_Inferred) return;

			int thickness = 4;
			ofSetColor(color);

			if (ts1 == TrackingState_Inferred || ts2 == TrackingState_Inferred) {
				thickness = 2;
				//ofSetColor(0, 128, 0); // TODO set this to modified value of color
			}

			ofSetLineWidth(thickness);
			ofDrawLine(pJointPoints[joint0], pJointPoints[joint1]);
		}

		//----------
		void Body::drawProjectedHand(HandState handState, ofVec2f & handPos){
			ofColor color;
			switch (handState)
			{
			case HandState_Unknown: case HandState_NotTracked:
				return;
			case HandState_Open:
				color = ofColor(0, 255, 0, 80);
				break;
			case HandState_Closed :
				color = ofColor(255, 255, 0, 80);
				break;
			case HandState_Lasso:
				color = ofColor(0, 255, 255, 80);
				break;
			}
			ofEnableAlphaBlending();
			ofSetColor(color);
			ofDrawCircle(handPos, 50);
			ofDisableAlphaBlending();
		}
	}
}

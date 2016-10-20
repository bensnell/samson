#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxXmlSettings.h"
#include "ofxFaceTracker.h"
#include "ofxAssimpModelLoader.h"
#include "ofxSyphon.h"
#include "ofxBlackmagic.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
    
    // ---------- MAIN PIPELINE ----------
    
    // -----------------------------------
    // ---------- UPDATE VIDEO -----------
    // -----------------------------------
    
    // update the input stream
    bool updateSource();        // update the stream we're drawing from
    int srcW, srcH;             // source dimensions
    
        bool updateVideo();
            ofVideoPlayer player;// holds current (original) frame
    
        bool updateMagic();
            ofxBlackMagic magic;// black magic input
    
        ofPixels fullFrame;     // full frame of video we're focusing on
        ofPixels thisFrame;     // copy of player cropped to select viewport (cv this!)
    

    // -----------------------------------
    // ----------- ATTEMPT CV ------------
    // -----------------------------------
    
    // perform computer vision on the video frame
    bool attemptCV();
    bool _attemptCV();          // helper function
    
    // ------- STAGE 1 --------
    
    bool splitScreen();         // test whether trump is in frame
        void setupSplitScreen();
    
    bool fullScreen();          // test whether trump is in full frame
        void setupFullScreen();
    
    
    // ------- STAGE 2 --------
    
    bool findFace();            // find trump's face
        ofxFaceTracker ft;
        vector<ofVec2f> facePts;// image face points
    
    bool extractForeground();   // extract trump's outline
        ofxCv::ContourFinder ctr;
        cv::Mat channels[3];    // thisFrame in mat form
        cv::Mat fgnd;           // foreground (white)
        ofPolyline trumpOutline;// outline of trump (largest foreground)

    // ------- STAGE 3 --------
    
    bool removeHair();          // remove trump's hair (produces two masks)
        ofxCv::ContourFinder ctrHair;
        ofPolyline hairOutline; // outline of hair
        ofColor backgroundColor;
        ofImage hairFiller;     // trump's hair mask (in background color)
        ofImage foreheadBlur;   // trump forehead blur (blended face mask)

    bool renderBald();          // render a bald model in correct orientation
        ofPolyline saggitalCut; // chin to top of head
        ofPolyline horizontalCut;// ear to ear
        ofPolyline eyeToEarLeft;// eye to ear line for calc intersection
        ofPolyline eyeToEarRight;// eye to ear line for calc intersection
        ofVec2f headCentroid;
        ofVec2f headCentroidAvg;// smoothed centroid of head (projected to 2D)
        ofImage baldModel;

    // -----------------------------------
    // --------- COMPOSITE FRAME ---------
    // -----------------------------------
    
    // composite the new frame
    void compositeAltered();    // all layers of altered images (bald trump)
    void compositeOriginal();   // the original frame
    
        ofTexture newFrame;

    // -----------------------------------
    // ---------- OUTPUT VIDEO -----------
    // -----------------------------------
    
    void outputSyphon();        // output the new frame through syphon
        ofxSyphonServer server;
    
    void initCVImages();
    
    
    // ------------- MISC -----------------
    
    vector<ofTexture> textures;
    
    // ------- FULL SCREEN TRAINING --------
    
    // average 9 x 16 pixel values (training set)
    vector<ofPixels> avgFullFrames;
        // 0 = Trump
        // 1 = Clinton
        // 2 = Moderator
    // number of frames used in each average
    vector<int> avgCounts;  // 3 total slots
    
    void setupFullScreenTraining();
    
    long getImageError(ofPixels &img1, ofPixels &img2);

    void addTrainingFrame(int index);
    
    // -------- SOURCE ---------
    
    void setupSource();
    
    bool setupMagic(); // for black magic
    // available modes: NTSC, NTSC 23.98, PAL, NTSC Progressive, PAL Progressive, 1080p23.98, 1080p24, 1080p25, 1080p29.97, 1080p30, 1080i50, 1080i59.94, 1080i60, 720p50, 720p59.94, 720p60
    
    // --------- VIDEO ---------
    
    bool loadVideo();
    void togglePlay();
    long frameNumber = -1;
    void drawVideo();
    
    void setupCTR();

    void setupThisFrameAndViewport();
    
    // -------- DRAW DEBUG --------
    
    void drawViewport();
    void drawForeground();
    void drawContours();
    void drawFaceTracker();
    void drawFaceMetrics();
    void drawModel();
    
    
    // -------- GEOMETRY ----------
    
    void loadModel();
    ofxAssimpModelLoader model;
    ofMesh mesh;
    
    
    // ------- RENDERING ---------
    
    void setupCamera();
    ofCamera cam;
    ofRectangle viewport;
    
    void setupLights();
    ofLight light1, light2;
    
    // --------- LOG -------------
    
    ofFile logFile;
    void setupLog();
    void log(string message);
    
    
    // ---------- GUI -----------
    
    void setupGui();
    
    ofxPanel panel;
    int paneloffsetY = 0;
    
    ofParameterGroup status;
    ofParameter<bool> isSplitScreen;
    ofParameter<bool> isFullScreen;
    ofParameter<bool> isFullScreenAndTrump;
    ofParameter<bool> isFaceFound;
    ofParameter<bool> isForegroundFound;
    ofParameter<bool> isHairRemoved;
    ofParameter<bool> isRenderedBald;
    
    ofParameterGroup global;
    ofParameter<bool> forceAllOff;  // stop all tracking
    ofParameter<bool> forceFullScreen;  // overrides everything
    ofParameter<int> delayFrames;

    ofParameterGroup inputSource;
    ofParameter<bool> isLiveSource; // if live, then black magic; if not, then video player
    ofParameter<int> liveWidth;
    ofParameter<int> liveHeight;
    ofParameter<int> liveFrameRate;
    ofParameter<bool> liveLowLatency;
    ofParameter<string> loadVideoName; // prerecorded file name
    
    ofParameterGroup fullScreenTraining;
    ofParameter<bool> bAttemptFullScreen;
    ofParameter<bool> bClearTraining; // flag to remove all trained samples
    ofParameter<int> trainingWidth;     // samples in x direction
    ofParameter<int> trainingHeight;    // samples in y
    ofParameter<string> trainingStatus;
    
    ofParameterGroup video;
    ofParameter<bool> bMute;
    ofParameter<bool> bDrawDebug;
    ofParameter<bool> bDrawOriginalVideo;
    ofParameter<bool> bDrawCompositeVideo;
    
    ofParameterGroup outputRender;
    ofParameter<bool> renderPlayer;
    ofParameter<bool> renderHairFiller;
    ofParameter<bool> renderBaldModel;
    ofParameter<bool> renderForeheadBlur; // (face mask)
    
    ofParameterGroup splitScreenIdentification;
    ofParameter<float> centerWidthDeviation;
    ofParameter<float> centerHeightFactor; // calculated from top
    ofParameter<int> nCenterSamples;
    ofParameter<float> maxDevPerSample;
    ofParameter<bool> bDrawSampleLocation;
    
    ofParameterGroup frame; // image extracted for processing
    ofParameter<int> frame_w; // for both split and full screen
    ofParameter<int> frame_h; // for both split and full screen
    ofParameter<int> frame_y; // for both split and full screen
    ofParameter<int> frame_x_ss; // for only split screen, since full screen means image is in center
    ofParameter<bool> bDrawViewport;

    
    // frame dimensions updated with locations dependending on split or full screen
    // INTERNAL TO APP
    int fr_x;
    int fr_y;
    int fr_w;
    int fr_h;
    
    ofParameterGroup faceTracking;
    ofParameter<bool> bDrawFacialLines;
    ofParameter<bool> bDrawFacialMesh;
    
    ofParameterGroup foregroundExtraction;
    ofParameter<float> colorMult; // multiplier by which image is thresholded
    ofParameter<int> threshFactor;
    ofParameter<int> blurSize;
    ofParameter<int> blurThresh;
    ofParameter<bool> bDrawForegroundScaled;
    ofParameter<bool> bDrawForegroundThresh;
    
    ofImage foregroundScaled, foregroundThresh;
    
    ofParameterGroup contours;
//    ofParameter<float> ctrMin; // not used right now
//    ofParameter<float> ctrMax; // ""
//    ofParameter<float> ctrThresh; // ""
//    ofParameter<bool> ctrHoles; // ""
    ofParameter<bool> bDrawAllContours;
    ofParameter<bool> bDrawHeadOutline;
    ofParameter<bool> bDrawHairOutline;
    // params for hair contours
    
    ofParameterGroup hairRemoval;
    ofParameter<float> earProjLength;
    ofParameter<float> browProjLength;
    ofParameter<int> hairFillerDilation; // negative becomes erode
    ofParameter<int> hairFillerBlur;
    
    ofParameterGroup faceMasking;
    ofParameter<float> dilForehead; // negative becomes erode
    ofParameter<float> blurForeheadFactor;
    
    ofParameterGroup faceMetrics;
    ofParameter<float> resampleLength;
    ofParameter<float> headCentroidEasing;
    ofParameter<int> sideHeadRadiusAverage;
    ofParameter<int> projectFromInnerEyeThresh;
    ofParameter<bool> bDrawCutLines;
    ofParameter<bool> bDrawHeadCentroid;
    ofParameter<bool> bDrawIntersectionLines;
    
    ofParameterGroup modelRendering;
    ofParameter<float> lightAttenuation;
    ofParameter<int> modelScale;
    ofParameter<int> modelYOffset;
    ofParameter<string> modelName;
    ofParameter<bool> bDrawModel;
    ofParameter<bool> bDrawAxes;
    
    ofParameterGroup windowSettings; // don't do anything when live -- change in xml
    ofParameter<int> windowWidth;
    ofParameter<int> windowHeight;
    
    void updateMagicLatencyMode(bool &lowLatency);
    void changeSource(bool &liveSource);
    void clearTrainingData(bool &clearData);
    
    
    // --------- UTILS ----------
    
    void drawDebug();
    
    void printPoint(string label, ofVec3f point);
    
    void printPolyline(string label, ofPolyline line);
    
    ofVec3f getCentroid3D(ofPolyline line);
    
    // from d1 to d2 starting at origin with length
    ofVec2f getProj(ofVec2f o, ofVec2f d1, ofVec2f d2, float l);
    
    // intersection between 2d closed polyline and straight line
    // start of straightline should be inside and end of straightline should be outside
    bool getLinePolylineIntersection(ofPolyline & straightLine, ofPolyline & closedPolyline, ofVec2f & intersectionPt, float sampleLength);
    
    void drawPolyline(ofPolyline line, bool bPoints, ofColor color, float pointRadius = 3, float lineWidth = 1);
    
    void drawPoint(ofVec2f point, ofColor color, float radius);
    
    void huntForBlendFunc(int period, int defaultSid, int defaultDid);
    
    void drawAxes(float scale);
    
    ofPolyline tmpl, tmpr;
    
};

#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetVerticalSync(true);
    ofEnableAlphaBlending();

    // load settings
    setupGui();
    
    // setup window size
    ofSetWindowShape(windowWidth, windowHeight);
    ofSetWindowTitle("Baldstreaming Trump");
    
    // setup source
    setupSource();
    setupFullScreenTraining();

    // setup cv
    ft.setup();                 // face tracker
    setupCTR();                 // contours
    initCVImages();             // composited layers

    // setup rendering
    setupCamera();
    setupLights();
    loadModel();
    
    // setup output
    setupLog();                 // log error file
    server.setName("Samson");   // syphon
    
}

//--------------------------------------------------------------
void ofApp::update(){

    // update video and put the new frame, if there is one, in thisFrame
    if (updateSource()) {
        
        // attempt to perform CV on the image to find trump's face, extract his figure and hair, and render a bald model in his place
        if (attemptCV()) {
            
            // composite a new, altered frame with the image manipulations
            compositeAltered();
            
        } else {
            
            // reproduce the original frame for output
            compositeOriginal();
        }
        
        // make this new frame available through syphon
        outputSyphon();
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(150);
    
    drawVideo();
    
    // ------------------ DEBUG --------------------
    
    drawViewport();
    
    drawForeground();
    
    drawContours();
    
    drawFaceTracker();
    
    drawFaceMetrics();
    
    drawModel();
    
    drawDebug();
}

//--------------------------------------------------------------
void ofApp::initCVImages() {
    
    // initialize images
    // player                                           //   A
    hairFiller.setImageType(OF_IMAGE_COLOR_ALPHA);      // + B
    baldModel.setImageType(OF_IMAGE_COLOR_ALPHA);       // + C
    foreheadBlur.setImageType(OF_IMAGE_COLOR_ALPHA);    // + D
    // newFrame                                         // = E
}

//--------------------------------------------------------------
bool ofApp::attemptCV() {
    
    // don't do anything if we're not going to try
    if (forceAllOff) {
        isSplitScreen = false;
        isFullScreen = false;
        isFullScreenAndTrump = false;
        isFaceFound = false;
        isForegroundFound = false;
        isHairRemoved = false;
        isRenderedBald = false;
        
        return false;
    }
    
    if (splitScreen()) {            // attempt to find a split screen shot
        isSplitScreen = true;
        isFullScreen = false;
        isFullScreenAndTrump = false;
        
        return _attemptCV();
        
    } else if (fullScreen()) {      // attempt to identify a full screen shot
        isFullScreen = true;
        isFullScreenAndTrump = true;
        isSplitScreen = false;
        
        return _attemptCV();
        
    } else {                        // no trump identified
        isSplitScreen = false;
        if (bAttemptFullScreen) {
            isFullScreen = true;
        } else {
            isFullScreen = false;
        }
        isFullScreenAndTrump = false;
        isFaceFound = false;
        isForegroundFound = false;
        isHairRemoved = false;
        isRenderedBald = false;
        return false;
    }
}

//--------------------------------------------------------------
bool ofApp::_attemptCV() {
    
    // attempt to find a face
    if (findFace()) {
        isFaceFound = true;
        
        // attempt to extract the foreground of the image
        if (extractForeground()) {
            isForegroundFound = true;
            
            // attempt to remove the hair of our selected individual
            if (removeHair()) {
                isHairRemoved = true;
                
                // attempt to render a bald head
                if (renderBald()) {
                    isRenderedBald = true;
                    return true;
                } else {
                    isRenderedBald = false;
                    return false;
                }
            } else {
                isHairRemoved = false;
                isRenderedBald = false;
                return false;
            }
        } else {
            isForegroundFound = false;
            isHairRemoved = false;
            isRenderedBald = false;
            return false;
        }
    } else {
        isFaceFound = false;
        isForegroundFound = false;
        isHairRemoved = false;
        isRenderedBald = false;
        return false;
    }
}

//--------------------------------------------------------------
bool ofApp::updateSource() {
    
    if (isLiveSource) {
        
        // update the black magic stream
        return updateMagic();
        
    } else {
        
        // update the video stream
        return updateVideo();
    }
}

//--------------------------------------------------------------
bool ofApp::updateMagic() {
    
    if (magic.update()) {
        
        // set the source media dimensions
        srcW = liveWidth;
        srcH = liveHeight;
        
        // set the full frame we're using to this frame
        fullFrame = magic.getColorPixels();
        
        // increment frame number
        frameNumber++;
        
        return true;
    } else {
        return false;
    }
}

//--------------------------------------------------------------
bool ofApp::updateVideo() {
    
    player.setVolume(bMute ? 0 : 1.0);
    
    player.update();
    if (player.isFrameNew()) {
        
        // set the source media dimenisons
        srcW = player.getWidth();
        srcH = player.getHeight();
        
        // copy player's pixels to generic fullFrame
        fullFrame = player.getPixels();
        
        // increment frame number
        frameNumber++;
        
        return true;
    } else {
        return false;
    }
}

//--------------------------------------------------------------
bool ofApp::splitScreen() {
    
    // if we're forcing full, then return false
    if (forceFullScreen) return false;

    
    // otherwise, try to find a split screen
    
    // get the center line of the video
    ofPixels center;
    if (isLiveSource) {
        magic.getColorPixels().cropTo(center, srcW * centerWidthDeviation, 0, 1, srcH * centerHeightFactor);
    } else {
        player.getPixels().cropTo(center, srcW * centerWidthDeviation, 0, 1, srcH * centerHeightFactor);
    }
    center.setImageType(OF_IMAGE_GRAYSCALE);
    // resize to the number of samples
    // prevent resizing to larger than image
    int nSamples = CLAMP((int)nCenterSamples, 0, int(srcH * centerHeightFactor));
    center.resize(1, nSamples);
    
    // find the average difference from pixel to pixel
    int sumDiff = 0;
    for (int i = 0; i < nSamples; i++) {
        // compare with index halfway ahead
        int comparisonIndex = (i + nSamples / 2) % nSamples;
        sumDiff += abs(center[i] - center[comparisonIndex]);
    }
    float avgDiff = (float)sumDiff / float(nSamples);
    
    // if the difference is less than our threshold, we have a split screen
    if (avgDiff <= maxDevPerSample) {
        
        // setup the splitscreen
        setupSplitScreen();
        return true;
        
    } else {
//        log("Frame # " + ofToString(frameNumber) + " is not a split screen. Aborting operations on this frame.");
        return false;
    }
}

//--------------------------------------------------------------
void ofApp::setupSplitScreen() {
    
    // update frame dimensions
    fr_x = CLAMP((int)frame_x_ss, 0, srcW);
    fr_y = CLAMP((int)frame_y, 0, srcH);
    fr_w = CLAMP((int)frame_w, 1, srcW - fr_w);
    fr_h = CLAMP((int)frame_h, 1, srcH - fr_y);

    setupThisFrameAndViewport();
}

//--------------------------------------------------------------
void ofApp::setupFullScreenTraining() {
    
    // set the lists to the correct size
    avgCounts.resize(3, 0);
    avgFullFrames.resize(3);
    
    // reset stats
    trainingStatus = "";
    
    // set force to false
    forceFullScreen = false;
}

//--------------------------------------------------------------
bool ofApp::fullScreen() {
    
    // if we're forcing fullscreen, then just return true
    if (forceFullScreen) {
        
        setupFullScreen();
        return true;
    }
    
    // otherwise, attempt to identify someone in the fullscreen image
    
    // if we aren't trying to do fullscreen, don't continue
    if (!bAttemptFullScreen) return false;
    
    // if we haven't trained images, don't continue
    int total = 0;
    bool comparisonFrames[avgCounts.size()];
    int trainedFrames = 0;
    for (int i = 0; i < avgCounts.size(); i++) {
        total += avgCounts[i];
        trainedFrames += (avgCounts[i] > 0) ? 1 : 0;
        comparisonFrames[i] = (avgCounts[i] > 0) ? true : false;
    }
    if (total == 0) return false;
    
    // at this point, we have at least one trained image. If we only have one and that one is trump, then we'll return true
    if (trainedFrames == 1 && comparisonFrames[0] == true) {
        
        // we are going to do cv on this frame, so copy the frame to thisFrame
        setupFullScreen();
        
        return true;
    }
    
    // at this point, there are more than one trained images, so for each one, compute how close it is to the current render window
    // first, get the current window in a resampled size
    ofPixels originalFrame;
    originalFrame.allocate(trainingWidth, trainingHeight, 3);
    // fill this sample with the pixels from the appropriate stream
    if (isLiveSource) {
        magic.getColorPixels().resizeTo(originalFrame);
    } else {
        player.getPixels().resizeTo(originalFrame);
    }
    // get rid of any alpha channel
    originalFrame.setNumChannels(3);
    
    // then we'll calculate the error of originalFrame with respect to each trained image
    long maxError = 255*255 * originalFrame.getWidth() * originalFrame.getHeight() * originalFrame.getNumChannels() + 1;
    int closestIndex = -1;
    for (int i = 0; i < avgCounts.size(); i++) {
        if (!comparisonFrames[i]) continue;
        
        // compare this frame with the training data
        long error = getImageError(originalFrame, avgFullFrames[i]);
        if (error == -1) {
            log("Training images and sample image are not the same size or not the same number of channels. Aborting fullscreen.");
            return false;
        }
        
        // if the error is less than the max, this is the new closest image
        if (error < maxError) {
            maxError = error;
            closestIndex = i;
        }
    }
    if (closestIndex == -1) {
        log("Could not find closest index in full screen. This shouldn't have happened.");
        return false;
    }
    
    // we've found the closest image. Check if it's trump
    if (closestIndex == 0) {
        
        // we found trump! copy his pixel data into thisFrame
        setupFullScreen();
        
        return true;
        
    } else {
        // we found someone else. Don't do anything
        return false;
    }
}

//--------------------------------------------------------------
void ofApp::setupFullScreen() {
    
    // update frame dimensions
    int tmpWidth = CLAMP((int)frame_w, 1, srcW);
    fr_x = CLAMP(srcW/2 - tmpWidth/2, 0, srcW);
    fr_y = CLAMP((int)frame_y, 0, srcH);
    fr_w = CLAMP((int)tmpWidth, 1, srcW - fr_x); // extraneous
    fr_h = CLAMP((int)frame_h, 1, srcH - fr_y);
    
    setupThisFrameAndViewport();
}

//--------------------------------------------------------------
void ofApp::setupThisFrameAndViewport() {
    
    // copy pixels from fullFrame to thisFrame
    fullFrame.cropTo(thisFrame, fr_x, fr_y, fr_w, fr_h);
    thisFrame.setNumChannels(3);
    
    // set the viewport dimensions
    viewport.set(fr_x, fr_y, fr_w, fr_h);
}

//--------------------------------------------------------------
long ofApp::getImageError(ofPixels &img1, ofPixels &img2) {
    
    // check if they're the same size?
    if (img1.getWidth() != img2.getWidth() || img1.getHeight() != img2.getHeight() || img1.getNumChannels() != img2.getNumChannels()) {
        return -1;
    }
    
    // sum the squared error of every pixel
    int nValues = img1.size();
    long sumError = 0;
    for (int i = 0; i < nValues; i++) {
        int diff = (img1[i] - img2[i]);
        sumError += diff*diff;              // could do abs value to speed up?
    }
    return sumError;
}

//--------------------------------------------------------------
bool ofApp::findFace() {
    
    // update the face tracker
    ft.update(toCv(thisFrame));
    
    if (ft.getFound()) {
        
        // update the face points
        facePts = ft.getImagePoints();
        
        return true;
        
    } else {
        
        log("Aborting frame # " + ofToString(frameNumber) + ". No faces found.");
        return false;
    }
}

//--------------------------------------------------------------
bool ofApp::extractForeground() {
    
    // split the image into its respective RGB channels
    split(toCv(thisFrame), channels); // split channels
    
    
    // ------- GET THE FOREGROUND OF THE IMAGE ---------
    Mat redFloat, blueFloat, blueToRed, blueToRedScaled, blueToRedChars, blueToRedThresh, blueToRedInv, blueToRedBlur, normalized;
    // dividing red by blue gets the innards of the face. we really want to highlight the inverse of the background, so we'll divide blue by red
    channels[0].convertTo(redFloat, CV_32FC1);
    channels[2].convertTo(blueFloat, CV_32FC1);
    cv::divide(blueFloat, redFloat, blueToRed);
    // scale the data up
    // the lowest value is about zero; the highest ranges between 5 - 25
    cv::multiply(blueToRed, Scalar(float(colorMult)), blueToRedScaled); // MUST MAKE COLOR MULT A FLOAT
    // CV::NORMALIZE DOES NOT WORK WELL WITH FLOATS -- DON'T USE IT HERE
    // convert mult 8 bit mat
    blueToRedScaled.convertTo(blueToRedChars, CV_8UC1);

    if (bDrawForegroundScaled) {
        toOf(blueToRedChars, foregroundScaled);
        foregroundScaled.update();
    }
    
    // threshold the foreground
    threshold(blueToRedChars, blueToRedThresh, threshFactor); // EXCESSIVE
    // get the inverse so trump is white
    cv:bitwise_not(blueToRedThresh, blueToRedInv);
    // Smooth the image so there aren't speckles everywhere
    blur(blueToRedInv, blueToRedBlur, blurSize);
    threshold(blueToRedBlur, fgnd, blurThresh);
    
    if (bDrawForegroundThresh) {
        toOf(fgnd, foregroundThresh);
        foregroundThresh.update();
    }
    
    
    // -------- GET CONTOURS OF FOREGROUND ----------
    ctr.findContours(fgnd);
    if (ctr.size() > 0) {
        // check to see if this blob is big enough
        int blobSizeFactor = 5; // one fifth
        unsigned int area =  ctr.getBoundingRect(0).area();
        if (area > (fr_w * fr_h / blobSizeFactor)) {
            // get its polyline
            trumpOutline = ctr.getPolyline(0);
        } else {
            log("Aborting frame # " + ofToString(frameNumber) + ". There is no outline big enough to be considered Trump.");
            return false;
        }
    } else {
        log("Aborting frame # " + ofToString(frameNumber) + ". The contour finder didn't find any outlines.");
        return false;
    }
    
    return true;
}

//--------------------------------------------------------------
bool ofApp::removeHair() {
    
    // includes getting a hair filler and a face mask
    
    
    // --------- GET THE LINES FROM EYES TO EARS ----------
    
    // get the vector from the left eye to the left ear
    ofVec2f l_eyeo = facePts[36]; // outer
    ofVec2f l_eyei = facePts[39]; // inner
    ofVec2f l_ear = facePts[0];
    // get the projection from the left ear outward away from the face
    // if the left ear is to the right of the left eye, then use the eyes as the vector direction
    ofVec2f l_proj = (l_ear.x < l_eyeo.x) ?
        l_ear + (l_ear - l_eyeo).getScaled(earProjLength) :
        l_eyeo + (l_eyeo - l_eyei).getScaled(earProjLength);
    
    // get the vector from the right eye to the right ear
    ofVec2f r_eyeo = facePts[45];
    ofVec2f r_eyei = facePts[42];
    ofVec2f r_ear = facePts[16];
    // get the projection
    ofVec2f r_proj = (r_ear.x > r_eyeo.x) ? // greater this time
        r_ear + (r_ear - r_eyeo).getScaled(earProjLength) :
        r_eyeo + (r_eyeo - r_eyei).getScaled(earProjLength);
    
    
    // ----------- GET HAIRLINE --------------
    
    // get the poyline of trump's hairline (from l_proj to r_proj)
    ofPath hairline;
    hairline.lineTo(l_proj);
    hairline.lineTo(getProj(facePts[17], facePts[36], facePts[17], browProjLength));
    hairline.lineTo(getProj(facePts[18], facePts[37], facePts[18], browProjLength));
    hairline.lineTo(getProj(facePts[19], facePts[38], facePts[19], browProjLength));
    hairline.lineTo(getProj(facePts[20], facePts[39], facePts[20], browProjLength));
    hairline.lineTo(getProj(facePts[21], facePts[39], facePts[21], browProjLength));
    hairline.lineTo(getProj(facePts[22], facePts[42], facePts[22], browProjLength));
    hairline.lineTo(getProj(facePts[23], facePts[42], facePts[23], browProjLength));
    hairline.lineTo(getProj(facePts[24], facePts[43], facePts[24], browProjLength));
    hairline.lineTo(getProj(facePts[25], facePts[44], facePts[25], browProjLength));
    hairline.lineTo(getProj(facePts[26], facePts[45], facePts[26], browProjLength));
    hairline.lineTo(r_proj);
    
    
    // -------- GET MASK OF ALL ABOVE HAIR, INCLUSIVE -------
    
    // create a closed shape that represents everything to subtract from trump (his hair, but not his forehead)
    ofPath hairAndBackground = hairline;
    hairAndBackground.lineTo(
                             hairAndBackground.getOutline()[0].getVertices().back().x + fr_w,
                             hairAndBackground.getOutline()[0].getVertices().back().y);
    hairAndBackground.lineTo(
                             hairAndBackground.getOutline()[0].getVertices().back().x,
                             hairAndBackground.getOutline()[0].getVertices().back().y - fr_h);
    hairAndBackground.lineTo(
                             hairAndBackground.getOutline()[0].getVertices().front().x - fr_w,
                             hairAndBackground.getOutline()[0].getVertices().front().y - fr_h);
    hairAndBackground.lineTo(
                             hairAndBackground.getOutline()[0].getVertices().front().x - fr_w,
                             hairAndBackground.getOutline()[0].getVertices().front().y);
    hairAndBackground.close();
    
    // draw this closed shape to an fbo to get a mask of everything above his hair, inclusive
    hairAndBackground.setFilled(true);
    hairAndBackground.setFillColor(ofColor(255, 255, 255));
    
    ofFbo fbo;
    fbo.allocate(fr_w, fr_h, GL_RGB); // don't actually need RGB; just R
    
    fbo.begin();
    ofClear(0, 0, 0);
    ofFill();
    hairAndBackground.draw();
    fbo.end();
    
    // get the pixels from the fbo
    ofPixels hairAndAbove;
    fbo.readToPixels(hairAndAbove);
    hairAndAbove.setNumChannels(1); // reduce 3 channels to 1
    
    
    // --------------- GET ALL POSSIBLE HAIRS -------------
    
    // boolean union of (trump / foreground) and (trump's hair & above) to just get his hair
    Mat hairAA = toCv(hairAndAbove); // hair and above (in mat form)
    Mat hairAll; // all possible hairs (may include fingers)
    bitwise_and(hairAA, fgnd, hairAll);
    
    
    // ------------------- DILATE HAIR --------------------
    
    // make it a bit bigger or smaller to cover less or more of his face (this should really be done after extracting this contour
    
    Mat hairAllDil;
    if (hairFillerDilation >= 0) {
        // dilate
        cv::dilate(hairAll, hairAllDil, cv::Mat(), cv::Point(-1, -1), hairFillerDilation);
    } else {
        // erode
        float hairFillerErosion = abs(hairFillerDilation);
        cv::erode(hairAll, hairAllDil, cv::Mat(), cv::Point(-1, -1), hairFillerErosion);
    }
    
    
    // ----------------- GET ONE HAIR ---------------------
    
    // MIGHT BE ABLE TO REMOVE THIS:
    
    // go through all contours found in hairAll to get the one closest one to the eyes
    ctrHair.findContours(hairAllDil);
    // if no hair is found, return from this function
    if (ctrHair.size() == 0) {
        log("Aborting frame # " + ofToString(frameNumber) + ". There are no hairs found to remove.");
        return false;
    }
    // find point between the brows
    ofVec2f browCenter = (facePts[21] + facePts[22]) / 2;
    int closestIndex = 0;
    int minDist = fr_w + fr_h; // manhattan distances
    for (int i = 0; i < ctrHair.size(); i++) {
        cv::Point2f hairCentroid = ctrHair.getCentroid(i);
        int dist = abs(browCenter.x - hairCentroid.x) + abs(browCenter.y - hairCentroid.y);
        if (dist < minDist) {
            minDist = dist;
            closestIndex = i;
        }
    }
    // get this hair outline
    hairOutline = ctrHair.getPolyline(closestIndex);

    
    // --------- FIND THE MEDIAN BACKGROUND COLOR ----------
    
    // Now, find the median background color to use
    ofPixels corn_l, corn_r;
    thisFrame.cropTo(corn_l, 0, 0, fr_w/3, fr_h/2);
    thisFrame.cropTo(corn_r, fr_w * 2 / 3, 0, fr_w/3, fr_h/2);
    int sample = 10;
    corn_l.resize(sample, sample);
    corn_r.resize(sample, sample);
    int nElems = sample * sample * 2;
    int nElemsHalf = sample * sample;
    int red[nElems];
    int green[nElems];
    int blue[nElems];
    for (int i = 0; i < nElemsHalf; i++) {
        red[i] = corn_l[3 * i];
        green[i] = corn_l[3 * i + 1];
        blue[i] = corn_l[3 * i + 2];
        
        red[nElemsHalf + i] = corn_r[3 * i];
        green[nElemsHalf + i] = corn_r[3 * i + 1];
        blue[nElemsHalf + i] = corn_r[3 * i + 2];
    }
    sort(red, red + nElems);
    sort(green, green + nElems);
    sort(blue, blue + nElems);
    
    backgroundColor = ofColor(red[nElemsHalf],
                              green[nElemsHalf],
                              blue[nElemsHalf]);
    
    
    // -------------- GET SELECT HAIR MASK ---------------
    
    // draw this contour in a new fbo to get the hair mask
//    fbo.clear();
    ofFbo newfbo;
    newfbo.allocate(fr_w, fr_h, GL_RGB);
    newfbo.begin();
    ofClear(0, 0, 0);
    
    ofSetColor(255);
    ofBeginShape();
    for (int i = 0, end = hairOutline.size(); i < end; i++) {
        ofVertex(hairOutline.getVertices()[i]);
    }
    ofEndShape();
    
    newfbo.end();
    
    ofPixels newHair;
    newfbo.readToPixels(newHair);
    newHair.setNumChannels(1);
    Mat newHairMat = toCv(newHair);
    
    // blur the hair mat
    Mat hairBlur;
    ofxCv::blur(newHairMat, hairBlur, hairFillerBlur);

    // get a pixels object of all blue, then set the alpha channel to reflect the hair alpha mask
    ofPixels blueBack, alphaChannel;
    blueBack.allocate(fr_w, fr_h, 4);
    blueBack.setColor(ofColor(backgroundColor.r, backgroundColor.g, backgroundColor.b, 0));
    toOf(hairBlur, alphaChannel);
    blueBack.setChannel(3, alphaChannel);
    
    hairFiller.getPixels() = blueBack;
    hairFiller.update();
    
    
    // --------------------------------------------------------------------
    // --------------------------------------------------------------------
    // --------------------------------------------------------------------
    
    
    // --- GET A MASK OF TRUMP'S FACE WITH NO HAIR AND SMOOTHED FOREHEAD ---
    
    // get an image of everything above the hair, dilated by half of the browProjLength
    Mat blendLine, blendLineInv;
    if (dilForehead >= 0) {
        // dilate
        cv::dilate(hairAA, blendLine, cv::Mat(), cv::Point(-1, -1), browProjLength * dilForehead);
    } else {
        // erode
        float eroForehead = abs(dilForehead);
        cv::erode(hairAA, blendLine, cv::Mat(), cv::Point(-1, -1), browProjLength * eroForehead);
    }
    // invert this image to keep everything below the hairline (this will become the transparency to an image)
    cv::bitwise_not(blendLine, blendLineInv);
    // blur the image
    cv::Mat blurHairline;
    blur(blendLineInv, blurHairline, browProjLength * blurForeheadFactor);

    // combine these channels into one image
    Mat belowHairBlended[4];
    Mat imgBlurHairline;
    copy(channels[0], belowHairBlended[0]);
    copy(channels[1], belowHairBlended[1]);
    copy(channels[2], belowHairBlended[2]);
    copy(blurHairline, belowHairBlended[3]);
    merge(belowHairBlended, 4, imgBlurHairline);
    toOf(imgBlurHairline, foreheadBlur);
    foreheadBlur.update();
    
    return true;
}

//--------------------------------------------------------------
bool ofApp::renderBald() {
    
    // ------------ FIND SAGGITAL CUT -----------------
    
    // first, find the y coordinate at the center of the line from the chin to the top of the head
    ofVec2f aboveHead = getProj(facePts[8], facePts[8], facePts[27], fr_h);
    ofPolyline chinToScalp;
    chinToScalp.addVertex(facePts[8].x, facePts[8].y);
    chinToScalp.addVertex(aboveHead);
    
    // find the intersection point with trump's outline
    ofVec2f topOfHead;
    bool intersect = getLinePolylineIntersection(chinToScalp, trumpOutline, topOfHead, resampleLength);
    if (!intersect) {
        log("Aborting frame # " + ofToString(frameNumber) + ". No intersections found with the top of the head");
        return false;
    }
    
    // find the point in the middle of the chin and scalp
    ofVec2f saggitalMid = (facePts[8] + topOfHead) / 2.;
    
    // for reference, make a line that represents the saggital cut
    saggitalCut.clear();
    saggitalCut.addVertex(facePts[8]);
    saggitalCut.addVertex(saggitalMid);
    saggitalCut.addVertex(topOfHead);
    
    
    // --------------- FIND HORIZONTAL CUT ---------------
    
    // find intersection on left side of head
    int leftTiltFactor = abs(facePts[0].x - facePts[36].x);
    int leftHeadProjLength = earProjLength + 2 * leftTiltFactor;
    // if the head is really turned toward the left, find the intersection starting at the inner eye toward the outer; otherwise, starting at the outer eye toward the side of the head
    ofVec2f leftHead = (leftTiltFactor < projectFromInnerEyeThresh) ?
        getProj(facePts[39], facePts[39], facePts[36], leftHeadProjLength) :
        getProj(facePts[36], facePts[36], facePts[0], leftHeadProjLength);
    eyeToEarLeft.clear();
    eyeToEarLeft.addVertex((leftTiltFactor < projectFromInnerEyeThresh) ?
                           facePts[39] :
                           facePts[36]);
    eyeToEarLeft.addVertex(leftHead);
    
    ofVec2f leftOfHead;
    intersect = getLinePolylineIntersection(eyeToEarLeft, trumpOutline, leftOfHead, resampleLength);
    if (!intersect) {
        log("Aborting frame # " + ofToString(frameNumber) + ". No intersections found with the left side of the head.");
        return false;
    }
    
    // average the points near this intersection to remove noise -- doesn't work very well
    ofVec2f avgLeftOfHead(0, 0);
    if (sideHeadRadiusAverage > 0) {
        unsigned int nearestIndex;
        trumpOutline.getClosestPoint(leftOfHead, &nearestIndex);
        for (int i = - sideHeadRadiusAverage, end = sideHeadRadiusAverage + 1; i < end; i++) {
            
            int thisIndex = (nearestIndex + i) % trumpOutline.size();
            avgLeftOfHead += trumpOutline.getVertices()[thisIndex];
        }
        avgLeftOfHead /= float(sideHeadRadiusAverage * 2 + 1);
    } else {
        avgLeftOfHead = leftOfHead;
    }
    
    // find intersection on right side of head
    int rightTiltFactor = abs(facePts[45].x - facePts[16].x);
    int rightHeadProjLength = earProjLength + 2 * rightTiltFactor;
    // if the head is really turned toward the right, find the intersection starting at the inner eye toward the outer; otherwise, starting at the outer eye toward the side of the head
    ofVec2f rightHead = (rightTiltFactor < projectFromInnerEyeThresh) ?
        getProj(facePts[42], facePts[42], facePts[45], rightHeadProjLength) :
        getProj(facePts[45], facePts[45], facePts[16], rightHeadProjLength);
    eyeToEarRight.clear();
    eyeToEarRight.addVertex((rightTiltFactor < projectFromInnerEyeThresh) ?
                            facePts[42] :
                            facePts[45]);
    eyeToEarRight.addVertex(rightHead);
    
    ofVec2f rightOfHead;
    intersect = getLinePolylineIntersection(eyeToEarRight, trumpOutline, rightOfHead, resampleLength);
    if (!intersect) {
        log("Aborting frame # " + ofToString(frameNumber) + ". No intersections found with the right side of the head.");
        return false;
    }
    
    // average the points near this intersection to remove noise -- doesn't work very well
    ofVec2f avgRightOfHead(0, 0);
    if (sideHeadRadiusAverage > 0) {
        unsigned int nearestIndex;
        trumpOutline.getClosestPoint(rightOfHead, &nearestIndex);
        for (int i = - sideHeadRadiusAverage, end = sideHeadRadiusAverage + 1; i < end; i++) {
            
            int thisIndex = (nearestIndex + i) % trumpOutline.size();
            avgRightOfHead += trumpOutline.getVertices()[thisIndex];
        }
        avgRightOfHead /= float(sideHeadRadiusAverage * 2 + 1);
    } else {
        avgRightOfHead = rightOfHead;
    }
    
    // find the point in the middle of these two intersections
    ofVec2f horiztonalMid = (avgRightOfHead + avgLeftOfHead) / 2.;
    
    // find the horizontal cut line
    horizontalCut.clear();
    horizontalCut.addVertex(avgLeftOfHead);
    horizontalCut.addVertex(horiztonalMid);
    horizontalCut.addVertex(avgRightOfHead);
    
    
    // ------------ FIND CENTROID OF HEAD ----------------
    
    // the y coord of the saggital and the x coord of the horizontal
    headCentroid = ofVec2f(horiztonalMid.x, saggitalMid.y);
    
    // ease the cenroid to remove jitter
    headCentroidAvg = headCentroidAvg * headCentroidEasing + headCentroid * (1 - headCentroidEasing);
    
    
    // ----------- RENDER BALD MODEL TO AN IMAGE ------------
    
    // set how bright the lights are
    light1.setAttenuation(lightAttenuation);
    light2.setAttenuation(lightAttenuation);
    
    ofFbo fbo;
    fbo.allocate(fr_w, fr_h, GL_RGBA);
    fbo.begin();
    ofClear(0, 0, 0, 0);
    
    cam.begin();
    
    ofEnableDepthTest();
    ofEnableAlphaBlending();
    ofEnableLighting();
    light1.enable(); light2.enable();
    
    // transform to correct location
    ofPushMatrix();
    ofScale(1, -1, 1); // remove the y flip
    ofTranslate(-fr_w/2., -fr_h/2.); // translate origin of viewport
    ofTranslate(headCentroidAvg);
    ofScale(1, -1, 1); // remove the y flip
    ofScale(-1, 1, 1);
    applyMatrix(ft.getRotationMatrix());
    
    // draw the model
    ofTranslate(0, modelYOffset);
    ofScale(modelScale, modelScale, modelScale);
    ofRotateX(-10);
    mesh.drawFaces();
    
    ofPopMatrix();
    
    light1.disable(); light2.disable();
    ofDisableLighting();
    ofDisableDepthTest();
    
    cam.end();
    
    fbo.end();
    ofDisableAlphaBlending(); // must call this AFTER fbo ends, or else transparency will not be blended
    
    // render fbo to image
    fbo.readToPixels(baldModel.getPixels());
    baldModel.update();
    
    return true;
}

//--------------------------------------------------------------
void ofApp::setupSource() {
    
    if (isLiveSource) {
        setupMagic();
    } else {
        if (!loadVideo()) {
            // if we can't load the video, refert to black magic
            log("Reverting to live stream.");
            setupMagic();
        }
    }
    
    thisFrame.setImageType(OF_IMAGE_COLOR);
}

//--------------------------------------------------------------
bool ofApp::loadVideo() {
    
    if (!player.load(loadVideoName)) {
        log("Could not load video \"" + loadVideoName + "\". Check to make sure the name is correct and that it's in the data folder.");
        return false;
    }
    player.setLoopState(OF_LOOP_NORMAL);
    player.setPaused(true);
    return true;
}

//--------------------------------------------------------------
bool ofApp::setupMagic() {
    
    // setup black magic
    magic.setup(liveWidth, liveHeight, liveFrameRate);
//    cam.setup(1920, 1080, 60); // works
//    cam.setup(1280, 720, 60); // works
    
    // set latency mode
    magic.setColorFrameCaptureMode( liveLowLatency ?
                                   ofxBlackMagic::ColorFrameCaptureMode::LOW_LATENCY :
                                   ofxBlackMagic::ColorFrameCaptureMode::NO_FRAME_DROPS);
    
    return true;
}

//--------------------------------------------------------------
void ofApp::drawVideo() {
    
    // draw the original video
    if (bDrawOriginalVideo) {
        if (isLiveSource) {
            magic.drawColor();
        } else {
            player.draw(0, 0);
        }
    }
    
    // draw the composite video frame (original image + bald trump)
    if (bDrawCompositeVideo && newFrame.isAllocated()) {
        newFrame.draw(0, 0); // sometimes will be original video
    }
}

//--------------------------------------------------------------
void ofApp::togglePlay() {
    
    if (player.isPaused()) {
        player.setPaused(false);
    } else {
        player.setPaused(true);
    }
}

//--------------------------------------------------------------
void ofApp::drawDebug() {
    
    if (!bDrawDebug) return;
    
    panel.setPosition(ofGetWidth() - panel.getWidth() - 10, 10 + paneloffsetY);
    panel.draw();
    
    stringstream ss;
    ss << setprecision(2) << setfill('0');
    ss << ofToString(ofGetFrameRate());
    ofDrawBitmapStringHighlight(ss.str(), 10, 20);
}

//--------------------------------------------------------------
void ofApp::compositeAltered() {
    
    ofFbo fbo; // must be rgb if sending out over syphon (if RGBA, then syphon shows black lines)
    fbo.allocate(srcW, srcH, GL_RGB);
    fbo.begin();
    ofClear(0, 0, 0);
    
    ofEnableAlphaBlending();
    
    if (renderPlayer) {
        if (isLiveSource) {
            magic.drawColor();
        } else {
            player.draw(0, 0);
        }
    }
    if (renderHairFiller) hairFiller.draw(fr_x, fr_y);
    if (renderBaldModel) baldModel.draw(fr_x, fr_y);
    if (renderForeheadBlur) foreheadBlur.draw(fr_x, fr_y);
    
    fbo.end();
    
    ofDisableAlphaBlending();
    
    newFrame = fbo.getTexture();
}

//--------------------------------------------------------------
void ofApp::compositeOriginal() {
    
    if (isLiveSource) {
        newFrame = magic.getColorTexture();
    } else {
        newFrame = player.getTexture();
    }
}

//--------------------------------------------------------------
void ofApp::outputSyphon() {
    
    // if the delay frames == 0, then clear the vector of textures and just output this texture
    if (delayFrames == 0) {
        
        textures.clear();
        
        server.publishTexture(&newFrame);
        
    } else {
        
        // there is a delay, so add this texture to the back of the vector
        textures.push_back(newFrame);
        
        while(textures.size() > delayFrames) {
            // publish
            server.publishTexture(&textures.front());
            
            // remove from vector
            textures.erase(textures.begin());
        }
    }
}

//--------------------------------------------------------------
void ofApp::addTrainingFrame(int index) {
    
    if (index >= avgFullFrames.size()) {
        log("Cannot add training frame at index greater than " + ofToString(avgFullFrames.size()) + ".");
        return;
    }
    
    // add this frame to the training set
    
    
    // first, get the data
    ofPixels tmpTrainingData;
    tmpTrainingData.allocate(trainingWidth, trainingHeight, 3);
    
    // fill this with the pixels from the appropriate stream
    if (isLiveSource) {
        magic.getColorPixels().resizeTo(tmpTrainingData);
    } else {
        player.getPixels().resizeTo(tmpTrainingData);
    }
    
    // set the number of channels to 3
    tmpTrainingData.setNumChannels(3);
    
    
    // add it to the appropriate index
    avgFullFrames[index] = tmpTrainingData;
    avgCounts[index]++;
    
    // keep track of stats for reference
    trainingStatus = trainingStatus + " " + ofToString(index);
    
    // should try to average it with prevoius readings....
}

//--------------------------------------------------------------
void ofApp::setupGui() {
    
    status.setName("Status");
    status.add(isSplitScreen.set("Split Screen", false));
    status.add(isFullScreen.set("Full Screen", false));
    status.add(isFullScreenAndTrump.set("Full Screen and Trump", false));
    status.add(isFaceFound.set("Face Found", false));
    status.add(isForegroundFound.set("Foregnd Found", false));
    status.add(isHairRemoved.set("Hair Removed", false));
    status.add(isRenderedBald.set("Rendered Bald", false));
    
    global.setName("Global Settings");
    global.add(forceAllOff.set("ALL OFF", false));
    global.add(forceFullScreen.set("FORCE FULL", false));
    global.add(delayFrames.set("Delay Frames", 0, 0, 300));
    
    inputSource.setName("Input Source");
    inputSource.add(isLiveSource.set("Live Source", true));
    inputSource.add(liveWidth.set("Live Src Width", 1280, 600, 1920));
    inputSource.add(liveHeight.set("Live Src Height", 720, 400, 1080));
    inputSource.add(liveFrameRate.set("Live Frame Rate", 60, 24, 60));
    inputSource.add(liveLowLatency.set("Low Latency", true));
    inputSource.add(loadVideoName.set("Load Video Name", "trump_test.mp4"));
    
    fullScreenTraining.setName("Full Screen");
    fullScreenTraining.add(bAttemptFullScreen.set("Attempt Full Screen", true));
    fullScreenTraining.add(bClearTraining.set("Clear Training Data", false));
    fullScreenTraining.add(trainingWidth.set("Sample Width", 16, 1, 45));
    fullScreenTraining.add(trainingHeight.set("Sample Height", 9, 1, 30));
    fullScreenTraining.add(trainingStatus.set("Stats", ""));
    
    video.setName("Video");
    video.add(bMute.set("Mute", true));
    video.add(bDrawDebug.set("Draw Debug", true));
    video.add(bDrawOriginalVideo.set("Draw Original", true));
    video.add(bDrawCompositeVideo.set("Draw Composite Video", false));

    outputRender.setName("Output Render");
    outputRender.add(renderPlayer.set("Original Player", true));
    outputRender.add(renderHairFiller.set("Hair Filler", true));
    outputRender.add(renderBaldModel.set("Bald Model", true));
    outputRender.add(renderForeheadBlur.set("Forehead Blur", true));
    
    splitScreenIdentification.setName("Identify Split Screen");
    splitScreenIdentification.add(centerWidthDeviation.set("Center Deviation", 0.5, 0.475, 0.525));
    splitScreenIdentification.add(centerHeightFactor.set("Center Height", 0.66667, 0, 1.0));
    splitScreenIdentification.add(nCenterSamples.set("Num Samples", 50, 3, 100));
    splitScreenIdentification.add(maxDevPerSample.set("Max Deviation", 5, 0, 50));
    splitScreenIdentification.add(bDrawSampleLocation.set("Draw Location", false));
    
    frame.setName("Frame");
    frame.add(frame_x_ss.set("X - SS Only", 0, 0, 1000));
    frame.add(frame_y.set("Y", 0, 0, 1000));
    frame.add(frame_w.set("Width", 500, 0, 1000));
    frame.add(frame_h.set("Height", 500, 0, 1000));
    frame.add(bDrawViewport.set("Draw Viewport", true));
    
    faceTracking.setName("Face Tracking");
    faceTracking.add(bDrawFacialLines.set("Draw Lines", true));
    faceTracking.add(bDrawFacialMesh.set("Draw Mesh", true));
    
    foregroundExtraction.setName("Extract Foreground");
    foregroundExtraction.add(colorMult.set("Img Thresh Scalar", 100, 0, 500));
    foregroundExtraction.add(threshFactor.set("Img Thresh Level", 230, 0, 255));
    foregroundExtraction.add(blurSize.set("Noise Rmvl Blur", 0, 0, 20));
    foregroundExtraction.add(blurThresh.set("Noise Rmvl Thresh", 128, 0, 255));
    foregroundExtraction.add(bDrawForegroundScaled.set("Draw Scaled Fgnd", false));
    foregroundExtraction.add(bDrawForegroundThresh.set("Draw Thresh Fgnd", false));
    
    contours.setName("Contours");
//    contours.add(ctrMin.set("Min Size", 200, 0, 1000));
//    contours.add(ctrMax.set("Max Size", 400, 0, 2000));
//    contours.add(ctrThresh.set("Threshold", 128, 0, 255));
//    contours.add(ctrHoles.set("bHoles", false));
    contours.add(bDrawAllContours.set("Draw All Contours", false));
    contours.add(bDrawHeadOutline.set("Draw Head Outline", false));
    contours.add(bDrawHairOutline.set("Draw Hair Outline", false));
    
    hairRemoval.setName("Hair Removal");
    hairRemoval.add(earProjLength.set("Ear Proj Len", 20, 1, 200));
    hairRemoval.add(browProjLength.set("Brow Proj Len", 10, 1, 100));
    hairRemoval.add(hairFillerDilation.set("Hair Fill Dilate", 5, -50, 50));
    hairRemoval.add(hairFillerBlur.set("Hair Fill Blur", 10, 0, 50));
    
    faceMasking.setName("Face Masking");
    faceMasking.add(dilForehead.set("Dilate 4hd Blur", 0.5, -2, 2));
    faceMasking.add(blurForeheadFactor.set("Blur 4hd Amt", 0.5, 0, 5));
    
    faceMetrics.setName("Face Metrics");
    faceMetrics.add(resampleLength.set("Itsct Precision", 2, 1, 20)); // lower = more precise
    faceMetrics.add(headCentroidEasing.set("Head Ctrd Smoothing", 0.9, 0, 1));
    faceMetrics.add(sideHeadRadiusAverage.set("Side Head Smoothing", 10, 0, 100));
    faceMetrics.add(projectFromInnerEyeThresh.set("Exd Itsct Proj Thrsh", 10, 0, 100));
    faceMetrics.add(bDrawCutLines.set("Draw Cut Lines", false));
    faceMetrics.add(bDrawHeadCentroid.set("Draw Head Ctrd", false));
    faceMetrics.add(bDrawIntersectionLines.set("Draw Itsct Lines", false));
    
    modelRendering.setName("Model Rendering");
    modelRendering.add(lightAttenuation.set("Light Attn", 1.5, 0, 3));
    modelRendering.add(modelYOffset.set("Y Offset", 30, -50, 50));
    modelRendering.add(modelScale.set("Scale", 80, 10, 200));
    modelRendering.add(modelName.set("Model Name", "bald_head_color5.ply"));
    modelRendering.add(bDrawModel.set("Draw Model", false));
    modelRendering.add(bDrawAxes.set("Draw Axes", false));
    
    windowSettings.setName("Window");
    windowSettings.add(windowWidth.set("Width", 1280));
    windowSettings.add(windowHeight.set("Height", 720));
    
    panel.setup();
    panel.setName("General Settings");
    panel.add(status);
    panel.add(global);
    panel.add(inputSource);
    panel.add(fullScreenTraining);
    panel.add(video);
    panel.add(outputRender);
    panel.add(splitScreenIdentification);
    panel.add(frame);
    panel.add(faceTracking);
    panel.add(foregroundExtraction);
    panel.add(contours);
    panel.add(hairRemoval);
    panel.add(faceMasking);
    panel.add(faceMetrics);
    panel.add(modelRendering);
    panel.loadFromFile("settings.xml");
    
    // add listeners
    liveLowLatency.addListener(this, &ofApp::updateMagicLatencyMode);
    isLiveSource.addListener(this, &ofApp::changeSource);
    bClearTraining.addListener(this, &ofApp::clearTrainingData);
}

//--------------------------------------------------------------
void ofApp::clearTrainingData(bool &clearData) {
    
    // reset bool
    bClearTraining = false;
    
    // clear all training data and counts
    avgFullFrames.clear();
    avgCounts.clear();
    
    // reset stats
    trainingStatus = "";
    
    setupFullScreenTraining();
}

//--------------------------------------------------------------
void ofApp::updateMagicLatencyMode(bool &lowLatency) {
    
    magic.setColorFrameCaptureMode( lowLatency ?
                                   ofxBlackMagic::ColorFrameCaptureMode::LOW_LATENCY :
                                   ofxBlackMagic::ColorFrameCaptureMode::NO_FRAME_DROPS);
}

//--------------------------------------------------------------
void ofApp::changeSource(bool &liveSource) {
    
    if (liveSource) {
        
        player.stop();
        player.close();
        
        log("Closing video player. Starting live source.");

        // clear most recent render
        newFrame.clear();
        
        setupMagic();
        
    } else {
        
        magic.close();
        
        newFrame.clear();
        
        log("Closing live source. Loading video from file.");
        
        if (!loadVideo()) {
            log("Could not load video. Starting live source again.");
            setupMagic();
        }
    }
}

//--------------------------------------------------------------
void ofApp::exit() {
    forceAllOff = false;
    magic.close();
    player.close();
    panel.saveToFile("settings.xml");
    logFile.close();
}

//--------------------------------------------------------------
void ofApp::setupLights() {
    
    light1.setAmbientColor(ofFloatColor(0.2));
    light2.setAmbientColor(ofFloatColor(0.2));
    light1.setPosition(-200, 300, 200);
    light2.setPosition(200, 350, 250);
}

//--------------------------------------------------------------
void ofApp::drawViewport() {
    
    if (bDrawSampleLocation) {
        ofPushStyle();
        ofNoFill();
        ofSetColor(255, 128, 0);
        ofDrawRectangle(srcW * centerWidthDeviation, 0, 1, srcH * centerHeightFactor);
        ofPopStyle();
    }
    
    if (bDrawViewport) {
        // draw the viewport in red
        ofPushStyle();
        ofNoFill();
        ofSetColor(255, 0, 0);
        ofDrawRectangle(viewport);
        ofPopStyle();
    }
}

//--------------------------------------------------------------
void ofApp::drawForeground() {
    
    if (bDrawForegroundScaled && foregroundScaled.isAllocated()) {
        foregroundScaled.draw(fr_x, fr_y);
    }
    
    if (bDrawForegroundThresh && foregroundThresh.isAllocated()) {
        foregroundThresh.draw(fr_x, fr_y);
    }
    
}

//--------------------------------------------------------------
void ofApp::drawContours() {
    
    if (!bDrawAllContours && !bDrawHeadOutline && !bDrawHairOutline) return;
    
    ofPushMatrix();
    ofTranslate(fr_x, fr_y);
    
    // draw all contours (head AND hair)
    if (bDrawAllContours) {
        
        ofPushStyle();
        
        ofSetLineWidth(3);
        ofSetColor(0, 255, 0);
        ctr.draw();
        
        ofSetLineWidth(1);
        ofSetColor(0, 0, 255);
        ctrHair.draw();
        
        ofPopStyle();
    }
    
    // draw trump's outline
    if (bDrawHeadOutline) {
        drawPolyline(trumpOutline, false, ofColor(255));
    }
    
    if (bDrawHairOutline) {
        drawPolyline(hairOutline, false, ofColor(255));
    }
    
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::drawFaceTracker() {
    
    if (bDrawFacialLines) {
        // draw the facial lines (and vertex indices)
        ofPushMatrix(); ofPushStyle();
        ofSetLineWidth(2);
        ofTranslate(fr_x, fr_y);
        ft.draw(true);
        ofPopMatrix(); ofPopStyle();
    }
    
    if (bDrawFacialMesh) {
        // draw the facial mesh (triangulated)
        
        cam.begin(viewport); // this puts the origin in this space at the center of the viewport
        
        ofPushMatrix();
        ofScale(1, -1, 1); // remove the y flip
        ofTranslate(fr_x -fr_w/2., - fr_y -fr_h/2.); // translate the origin to the origin of the viewport
        
        ofTranslate(ft.getPosition());
        ofScale(ft.getScale(), ft.getScale(), ft.getScale());
        applyMatrix(ft.getRotationMatrix());
        ft.getObjectMesh().drawWireframe();
        ofPopMatrix();

        cam.end();
    }
}

//--------------------------------------------------------------
void ofApp::drawFaceMetrics() {
    
    if (!bDrawCutLines && !bDrawHeadCentroid && !bDrawIntersectionLines) return;
    
    ofPushMatrix();
    ofTranslate(fr_x, fr_y);
    
    if (bDrawIntersectionLines) {
        // draw the lines for calculating intersections on both the right and left sides
        drawPolyline(eyeToEarLeft, true, ofColor(255, 200, 50));
        drawPolyline(eyeToEarRight, true, ofColor(255, 50, 200));
    }
    
    if (bDrawCutLines) {
        // draw the saggital cut line
        drawPolyline(saggitalCut, true, ofColor(255, 255, 0), 3, 1);
        
        // draw the horizontal cut line
        drawPolyline(horizontalCut, true, ofColor(0, 255, 0), 3, 1);
    }
    
    if (bDrawHeadCentroid) {
        // draw head's centroid
//        drawPoint(headCentroidAvg, ofColor(0, 255, 255, 200), 115);
        drawPoint(headCentroidAvg, ofColor(255), 5);
        drawPoint(headCentroid, ofColor(0), 2);
    }
    
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::drawModel() {
    
    if (!bDrawModel && !bDrawAxes) return;
    
    cam.begin(viewport); // this puts the origin in this space at the center of the viewport
    
    ofPushMatrix();
    ofScale(1, -1, 1); // remove the y flip
    ofTranslate(fr_x -fr_w/2., - fr_y -fr_h/2.); // translate the origin to the origin of the viewport
    
    // begin lights
    light1.setAmbientColor(ofFloatColor(0.2));
    light2.setAmbientColor(ofFloatColor(0.2));
    light1.setPosition(-200, 300, 200);
    light2.setPosition(200, 350, 250);
    light1.setAttenuation(lightAttenuation);
    light2.setAttenuation(lightAttenuation);
    
    ofEnableDepthTest();
    light1.enable(); light2.enable();
    ofEnableLighting();
    
    // apply transformations
    ofTranslate(headCentroidAvg);
    ofScale(1, -1, 1); // remove the y flip
    ofScale(-1, 1, 1);
    applyMatrix(ft.getRotationMatrix());
    
    // draw reference axes
    if (bDrawAxes) drawAxes(4);
    
    if (bDrawModel) {
        // model specific transformations:
        ofTranslate(0, modelYOffset);
        ofScale(modelScale, modelScale, modelScale);
        ofRotateX(-10);
        
        mesh.drawFaces();
    }
    
    // end lights
    ofDisableLighting();
    light1.disable(); light2.disable();
    ofDisableDepthTest();
    
    ofPopMatrix();
    cam.end();
}

//--------------------------------------------------------------
void ofApp::setupCamera() {
    
    cam.setFarClip(1000);
    cam.setNearClip(-1000);
    cam.enableOrtho();
}

//--------------------------------------------------------------
void ofApp::printPoint(string label, ofVec3f point) {
    
    cout << label << "\n";
    cout << "\t" << point.x << "\t" << point.y << "\t" << point.z << endl;
}

//--------------------------------------------------------------
ofVec3f ofApp::getCentroid3D(ofPolyline line) {
    
    int n = line.getVertices().size();
    
    if (n == 0) return ofVec3f(0, 0, 0);
    
    ofVec3f sum(0, 0, 0);
    for (int i = 0; i < n; i++) {
        sum += line.getVertices()[i];
    }
    return sum / float(n);
}

//--------------------------------------------------------------
void ofApp::printPolyline(string label, ofPolyline line) {
    
    int n = line.getVertices().size();
    
    cout << label << endl;
    for (int i = 0; i < n; i++) {
        cout << "\t" << line.getVertices()[i].x << "\t" << line.getVertices()[i].y << "\t" << line.getVertices()[i].z << endl;
    }
}

//--------------------------------------------------------------
void ofApp::setupCTR() {
    
    ctr.setSortBySize(true);
    ctr.setTargetColor(ofColor(255));
    ctr.setFindHoles(false);
    ctr.setMinAreaRadius(0); // just area?
    ctr.setMaxAreaRadius(600);
    ctr.setThreshold(128);
    
    ctrHair.setTargetColor(ofColor(255));
    ctrHair.setFindHoles(false);
    ctrHair.setMinAreaRadius(0);
    ctrHair.setMaxAreaRadius(600);
    ctrHair.setThreshold(128);
}

//--------------------------------------------------------------
ofVec2f ofApp::getProj(ofVec2f o, ofVec2f d1, ofVec2f d2, float l) {
    
    ofVec2f out = o + (d2 - d1).getScaled(l);
    return out;
}

//--------------------------------------------------------------
void ofApp::loadModel() {
    
    model.loadModel(modelName);
    mesh = model.getMesh(0);
}

//--------------------------------------------------------------
bool ofApp::getLinePolylineIntersection(ofPolyline & straightLine, ofPolyline & closedPolyline, ofVec2f & intersectionPt, float sampleLength) {
    
    // resample the straight line
    ofPolyline rs = straightLine.getResampledBySpacing(sampleLength);
    
    // make sure the polyline is closed
    closedPolyline.close();
    vector<ofPoint> & pts = rs.getVertices();
    
    //perform a binary search for the intersection point
    // assumes rs starts inside and ends outside
    int lo = 0;
    int hi = rs.size();
    int counter = 0;
    int nFlips = 0;
    bool prevInside;
    while (lo < hi) {
        
        int mid = (hi - lo) / 2 + lo;
        
        // whether we are inside or outside the closed polyline
        bool inside;
        
        if (closedPolyline.inside(pts[mid])) {
            inside = true;
            lo = mid + 1;
        } else {
            inside = false;
            hi = mid;
        }
        
        // keep track of whether we crossed an edge
        if (counter > 0 && inside != prevInside) nFlips++;
        prevInside = inside;
        counter++;
    }
    // lo and hi should be equal here
    if (nFlips > 0) {
        // we crossed a boundary, so return true
        intersectionPt = pts[lo];
        return true;
    } else {
        // we didn't cross a boundary, so return false
        return false;
    }
}

//--------------------------------------------------------------
void ofApp::drawPolyline(ofPolyline line, bool bPoints, ofColor color, float pointRadius, float lineWidth) {
    
    ofPushStyle();
    ofSetLineWidth(lineWidth);
    ofSetColor(color);
    line.draw();

    if (bPoints) {
        for (int i = 0; i < line.size(); i++) {
            ofDrawCircle(line.getVertices()[i], pointRadius);
        }
    }
    ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::drawPoint(ofVec2f point, ofColor color, float radius) {
    
    ofPushStyle();
    
    ofSetColor(color);
    ofDrawCircle(point, radius);
    
    ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::drawAxes(float scale) {

    ofPushMatrix(); ofPushStyle();
    ofScale(scale, scale, scale);
    
    ofSetColor(255);
    ofDrawBox(0, 0, 0, 30);
    
    ofSetColor(255, 0, 0);
    ofDrawBox(30, 0, 0, 15);
    
    ofSetColor(0, 255, 0);
    ofDrawBox(0, 30, 0, 15);
    
    ofSetColor(0, 0, 255);
    ofDrawBox(0, 0, 30, 15);

    ofPopMatrix(); ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::huntForBlendFunc(int period, int defaultSid, int defaultDid){
    // sets all possible combinations of blend functions,
    // changing modes every [period] milliseconds.
    
    // All checked out, works well.
    
    int sfact[] = {
        GL_ZERO,
        GL_ONE,
        GL_DST_COLOR,
        GL_ONE_MINUS_DST_COLOR,
        GL_SRC_ALPHA,
        GL_ONE_MINUS_SRC_ALPHA,
        GL_DST_ALPHA,
        GL_ONE_MINUS_DST_ALPHA,
        GL_SRC_ALPHA_SATURATE
    };
    
    int dfact[] = {
        GL_ZERO,
        GL_ONE,
        GL_SRC_COLOR,
        GL_ONE_MINUS_SRC_COLOR,
        GL_SRC_ALPHA,
        GL_ONE_MINUS_SRC_ALPHA,
        GL_DST_ALPHA,
        GL_ONE_MINUS_DST_ALPHA
    };
    
    
    
    glEnable(GL_BLEND);
    
    if ((defaultSid == -1) && (defaultDid == -1)) {
        
        int sid =  (ofGetElapsedTimeMillis()/(8*period))%9;
        int did =  (ofGetElapsedTimeMillis()/period)%8;
        glBlendFunc(sfact[sid], dfact[did]);
        // ofLog(OF_LOG_NOTICE, "SRC %d	DST %d\n", sid, did);
        printf("SRC %d	DST %d\n", sid, did);
        
    } else if (defaultDid == -1){
        
        int did =  (ofGetElapsedTimeMillis()/period)%8;
        glBlendFunc(sfact[defaultSid], dfact[did]);
        // ofLog(OF_LOG_NOTICE, "SRC %d	DST %d\n", defaultSid, did);
        printf("SRC %d	DST %d\n", defaultSid, did);
        
    } else if (defaultSid == -1){
        
        int sid =  (ofGetElapsedTimeMillis()/(8*period))%9;
        glBlendFunc(sfact[sid], dfact[defaultDid]);
        // ofLog(OF_LOG_NOTICE, "SRC %d	DST %d\n", sid, defaultDid);
        printf("SRC %d	DST %d\n", sid, defaultDid);
        
    } else {
        
        glBlendFunc(sfact[defaultSid], dfact[defaultDid]);
        
    }
    
}

//--------------------------------------------------------------
void ofApp::log(string message) {
    
//    cout << message << endl;
    
    logFile << message << "\n";
}

//--------------------------------------------------------------
void ofApp::setupLog() {
    
    logFile.open("logs/log_" + ofGetTimestampString() + ".txt", ofFile::WriteOnly);
    logFile << "Recording logs from Samson (Trump Balder) application at " << ofGetTimestampString() << "\n";
    logFile << "\n" << "-------------" << "\n\n";
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if (key == 'f') ofToggleFullscreen();
    if (key == ' ') togglePlay();
    if (key == 'd') bDrawDebug = !bDrawDebug;
    if (key == OF_KEY_DOWN) paneloffsetY += 10;
    if (key == OF_KEY_UP) paneloffsetY -= 10;
    
    // training keys
    if (key == 't') addTrainingFrame(0);    // add trump
    if (key == 'y') addTrainingFrame(1);    // add clinton
    if (key == 'u') addTrainingFrame(2);    // add moderator
    
    // force full screen
    if (key == 'p') forceFullScreen = true;
    if (key == 'o') forceAllOff = true;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

    if (key == 'p') forceFullScreen = false;
    if (key == 'o') forceAllOff = false;
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

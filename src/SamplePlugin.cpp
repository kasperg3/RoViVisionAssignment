#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

//Own includes:
#include <rw/kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>



using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std::placeholders;
//own ns
using namespace rw::math;
using namespace std;



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // now connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

    Image textureImage(300,300,Image::GRAY,Image::Depth8U);
    _textureRender = new RenderImage(textureImage);
    Image bgImage(0,0,Image::GRAY,Image::Depth8U);
    _bgRender = new RenderImage(bgImage,2.5/1000.0);
    _framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
    log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);

    // Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/kasper/RWworkspace/workcells/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);

    // Load Lena image
    Mat im, image;
    im = imread("/home/kasper/RWworkspace/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
    cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
    if(! image.data ) {
        RW_THROW("Could not open or find the image: please modify the file path in the source code!");
    }
    QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
    _label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
    // Add the texture render to this workcell if there is a frame for texture
    Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
    }
    // Add the background render to this workcell if there is a frame for texture
    Frame* bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
    }

    // Create a GLFrameGrabber if there is a camera frame with a Camera property set
    Frame* cameraFrame = _wc->findFrame("CameraSim");
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width,height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;
            // Create a frame grabber
            _framegrabber = new GLFrameGrabber(width,height,fovy);
            SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
            _framegrabber->init(gldrawer);
        }
    }
    }
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
    Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
    res.data = (uchar*)img.getImageData();
    return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_btn0){
        log().info() << "Button 0\n";
        // Set a new texture (one pixel = 1 mm)
        Image::Ptr image;
        image = ImageLoader::Factory::load("/home/kasper/RWworkspace/SamplePluginPA10/markers/Marker1.ppm");
        _textureRender->setImage(*image);
        image = ImageLoader::Factory::load("/home/kasper/RWworkspace/SamplePluginPA10/backgrounds/color1.ppm");
        _bgRender->setImage(*image);
        getRobWorkStudio()->updateAndRepaint();
    } else if(obj==_btn1){
        log().info() << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive())
            _timer->start(2000); // run 10 Hz
        else
            _timer->stop();
    } else if(obj==_spinBox){
        log().info() << "spin value:" << _spinBox->value() << "\n";
    }
}

Transform3D<> getTFromMat(vector<vector<double>> m, int lN){
    Vector3D<> posVec(m[lN][0]
                     ,m[lN][1]
                     ,m[lN][2]);

    RPY<> RPYVec(m[lN][3]
                ,m[lN][4]
                ,m[lN][5]);
    Rotation3D<> rot3D = RPYVec.toRotation3D();

    Transform3D<> trans(posVec,rot3D);
    return trans;
}
void TTMatrix(Transform3D<> T, Eigen::MatrixXf &m){
    int rows = 4;
    int columns = 4;
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < columns; j++){
            m(i,j) = T(i,j);
        }
    }
}
void getError(vector<vector<double>> vec, Transform3D<> current, Eigen::VectorXf &error, Transform3D<> camTWorld){


    double f = 823;
    double x = current.P()[0];
    double y = current.P()[1];
    double z = current.P()[2];

    cout << "x: " << x << endl;
    cout << "y: " << y << endl;


    Transform3D<> markerInWorld = getTFromMat(vec,0);
    Transform3D<> desired = camTWorld*markerInWorld;

    cout << "Transform current: " << current << endl;
    cout << "Transform desired: " << desired << endl;

    //Desired - current
    Vector3D<> dP = desired.P()-current.P();
    double dX = dP[0];
    double dY = dP[1];
    double dZ = dP[2];

    double dThetaZ = 0.5 * ((desired(2,1) * current(1,2)) - (desired(1,2) * (current(2,1))));
    double dThetaY = 0.5 * ((desired(0,2) * current(2,0)) - (desired(2,0) * (current(0,2))));
    double dThetaX = 0.5 * ((desired(1,0) * current(0,1)) - (desired(0,1) * (current(1,0))));

    double u = (f*x)/z;
    double v = (f*y)/z;

    cout << "dX: " << dX << endl;
    cout << "dY: " << dY << endl;
    cout << "dZ: " << dZ << endl;
    cout << "dthetaX: " << dThetaX << endl;
    cout << "dthetaY: " << dThetaY << endl;
    cout << "dthetaZ: " << dThetaZ << endl;


    double du = (-(f/z)*dX) + ((u/z)*dZ) + (((u*v)/f)*dThetaX) - ((pow(f,2) + pow(u,2))/f)*dThetaY + v*dThetaZ;
    double dv = (-(f/z)*dY) + ((v/z)*dZ) + ((pow(f,2) + pow(v,2))/f)*dThetaX - (((u*v)/f)*dThetaY) - u*dThetaZ;

    //double du = (f/z)*(-dX-z*dThetaY+y*dThetaZ)-((f*x)/pow(z,2))*(-dZ-y*dThetaX+x*dThetaY);
    //double dv = (f/z)*(-dY-x*dThetaZ+z*dThetaX)-((f*y)/pow(z,2))*(-dZ-y*dThetaX+x*dThetaY);

    error(0) = du;
    error(1) = dv;


}

void makeS(Rotation3D<> R, Eigen::MatrixXf &S){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            S(j,i) = R(i,j);
            S(j+3,i+3) = R(i,j);
        }
    }
}

void getImageJacobian(Transform3D<> camTrans, Eigen::MatrixXf &Jimg){
    int f = 823;
    double z = 0.5;
    int u = (f * camTrans(0,3))/z;
    int v = (f * camTrans(1,3))/z;

    Jimg(0,0) = (-f)/z;
    Jimg(0,1) = 0;
    Jimg(0,2) = u/z;
    Jimg(0,3) = (u*v)/f;
    Jimg(0,4) =-(pow(f,2)+pow(u,2))/f;
    Jimg(0,5) =v;

    Jimg(1,0) =0;
    Jimg(1,1) =(-f)/z;
    Jimg(1,2) =v/z;
    Jimg(1,3) =(pow(f,2)+pow(v,2))/f;
    Jimg(1,4) =-(u*v)/f;
    Jimg(1,5) =-u;
}

void JTMatrix(Jacobian J, Eigen::MatrixXf &m){
    int rows = J.size1();
    int columns = J.size2();
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < columns; j++){
            m(i,j) = J(i,j);
        }
    }
}




void addDq(Q &q, Eigen::VectorXf d_q ){
    int joints = 7;
    for(int i = 0; i < joints; i++){
       // cout << "addDq func: Before adding:Q" << i << ": " << q(i) << endl;
       // cout << "Q(i): " << q(i) << endl;
       // cout << "d_q(i): " << d_q(i) << endl;
        q(i) = q(i) + d_q(i);
       // cout << "addDq func: After adding:Q" << i << ": " << q(i) << endl;
    }
}


 Q algorithm2(Device::Ptr &device, State &state, WorkCell::Ptr &_wc, vector<vector<double>> desired){
    //init:
    double displacementEpsilon = 10;
    Q q = device->getQ(state);
    Eigen::VectorXf Dq(7);
    Eigen::MatrixXf A(2,7);
    Eigen::MatrixXf J(6,7);
    Eigen::MatrixXf JImage(2,6);
    Eigen::VectorXf Du(2);
    Eigen::MatrixXf S = Eigen::MatrixXf::Zero(6,6);
    Frame* worldFrame = _wc->getWorldFrame();
    MovableFrame* camFrame =(MovableFrame*) _wc->findFrame("CameraSim");
    Frame* baseFrame = device->getBase();
    MovableFrame* markerFrame = _wc->findFrame<MovableFrame>("Marker");
    Transform3D<> current = Kinematics::frameTframe(camFrame, markerFrame, state);
    Transform3D<> cTw = Kinematics::frameTframe(camFrame,worldFrame, state);

    for(int i = 0; i < 7; i++){
        cout << "first q" << i << " " << q(i) << endl;
    }

    //1: Compute difference d_u in current and Desired

    getError(desired, current, Du, cTw);
    cout << "Du: " << Du(0) << " & " << Du(1) << endl;
    //2: while difference d_u is > epsilon, do:
    //while( Du.norm() > displacementEpsilon){
    for(int x = 0; x < 10; x++){
        cout << "--------------------------" << endl;
        cout << "Norm of Du: " << Du.norm() << endl;

        //3: compute J(q)

        Jacobian tmpJ = device->baseJframe(camFrame,state);
        //Make Jacobian to Eigen matrix

        JTMatrix(tmpJ,J);
        cout <<"J: " << J << endl;

        //Compute J Img
        Transform3D<> camTMarker = Kinematics::frameTframe(camFrame, markerFrame, state);
        getImageJacobian(camTMarker, JImage);
        cout <<"Jimg: " << JImage << endl;


        //Compute S(q)
        Frame* baseFrame = device->getBase();
        Transform3D<> baseTCam = Kinematics::frameTframe(baseFrame,camFrame, state);
        Rotation3D<> R = baseTCam.R();
        makeS(R,S);
        cout <<"S: " << S << endl;


        //4: solve {J_img *  S(q) * J(q) * d_q = d_u} for d_q
        //Make the eq to Ax=b
        A = JImage * S * J;
        cout <<"A: " << A << endl;

       //Solve Ax=b
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Dq = svd.solve(Du);

        cout << "Dq: "<< Dq << endl;

        cout << "q_old:"<< "Q[7]{";
        for(int i = 0; i < 7; i++){
             cout << q(i);
             if(i != 6){
                 cout << ", ";
             }
        }
        cout << "}" << endl;

        //5: add d_q to q
        addDq(q,Dq);

        cout << "q_new:"<< "Q[7]{";
        for(int i = 0; i < 7; i++){
             cout << q(i);
             if(i != 6){
                 cout << ", ";
             }
        }
        cout << "}" << endl;

        //6: computre new T(q)
        device->setQ(q,state);

        current = Kinematics::frameTframe(camFrame,markerFrame,state);

        cTw = Kinematics::frameTframe(camFrame,worldFrame, state);
        //7: compute new d_u
        getError(desired, current, Du, cTw);
        cout << "new Du: " << Du(0) << " & " << Du(1) << endl;
    }
    return q;
}

vector<vector<double>> readMatFromFile(string str){
   //https://stackoverflow.com/questions/27789863/how-do-i-read-from-a-specific-line-in-a-file
   ifstream infile(str);
   vector<vector<double>> Numbers;            //Stores all the file's numbers
   string String;
   double a;
   while (getline(infile, String)) {
       vector<double> Line;                    //Stores each line's numbers
       stringstream Stream(String);
       while (Stream >> a)                  //Extracts numbers from 'Stream'
           Line.push_back(a);               //Each number is added to Line
       Numbers.push_back(Line);             //Each Line is added to Numbers
   }
   return Numbers;
}

void SamplePlugin::timer() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        Mat im = toOpenCVImage(image);
        Mat imflip;
        cv::flip(im, imflip, 0);

        // Show in QLabel
        QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 400;
        unsigned int maxH = 800;
        _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
    }

    //Create lua files to execute simulation
    vector<vector<double>> markerMotionSlow = readMatFromFile("/home/kasper/RWworkspace/SamplePluginPA10/motions/MarkerMotionSlow.txt");

    //writeLuaFile(markerMotionSlow, "");

    State state = _wc->getDefaultState();
    Device::Ptr devicePA10 = _wc->findDevice("PA10");
    Q q_start = Q(7, 0, -0.65,0,1.80,0,0.42,0);
    //Q q_start = Q(7, 0, -0.651, -0.231, 1.761, 0, 0.42, 0);
    devicePA10->setQ(q_start,state);
    Q q = devicePA10->getQ(state);
    cout << "Current Q: " << q(0) << ","<<q(1) << "," << q(2) << "," << q(3) << "," <<
                             q(4) << "," << q(5) << "," << q(6) << endl;

    //Find new q with newton raphson (algorithm 2 in robotics notes)
    q = algorithm2(devicePA10, state, _wc, markerMotionSlow);
    cout << "new Q: " << q(0) <<","<< q(1) << "," << q(2) << "," << q(3) << "," <<
                         q(4) << "," << q(5) << "," << q(6) << endl;

}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}




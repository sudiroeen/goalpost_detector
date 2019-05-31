#include "v9_goalpost_global/v9_goalpost_global.h"

Mat gambar;
Mat gambarClone;
Point titikStart;
bool afterDownBeforeUp = false;
Rect rectROI;


Mat imYX, imYY;

int hl = 0, sl = 0, vl = 0;
int hh = 179;
int sh = 255, vh = 255;


// #define DEBUG_PUTIH
GoalPostDetector::GoalPostDetector()
                ://KOLOM(640.0), BARIS(480.0),
                nh_(ros::this_node::getName()),
                it_(this->nh_)
{
    goal_pos_pub = nh_.advertise<geometry_msgs::Point>("goal_pos", 100);

    path_yaml = ros::package::getPath("v9_goalpost_global") + "/config/LookUpTable.yaml";
    
    get_frame_camera = it_.subscribe("/usb_cam_node/image_raw", 100, &GoalPostDetector::GetFrameCamCallback, this);

    kosong = bacaYAML(path_yaml.c_str());
    if(kosong.rows == 0)
        kosong = Mat::zeros(1, (64 << 12), CV_32SC1);
}

GoalPostDetector::~GoalPostDetector(){

}

Mat GoalPostDetector::bacaYAML(string namafile){
    FileStorage fs(namafile, FileStorage::READ);
    Mat temp;
    fs["LookUpTable"] >> temp;
    fs.release();

    return temp;
}

void GoalPostDetector::simpanTabel(Mat matrix, string namafile){
    FileStorage fs(namafile, FileStorage::WRITE);
    fs << "LookUpTable" << matrix;
    fs.release();
}

void GoalPostDetector::masukanMatrix(int nilai, Mat gambar, Rect kotak){
    int xrs, yrs, xrf, yrf;
    xrs = kotak.x;
    yrs = kotak.y;
    xrf = xrs + kotak.width;
    yrf = yrs + kotak.height;

    for(int xx=xrs+1; xx<xrf; xx++){
        for(int yy=yrs+1; yy<yrf; yy++){
            Vec3b pixel = gambar.at<Vec3b>(yy,xx);

            R = (int)((int)pixel[2] >> 2);
            G = (int)((int)pixel[1] >> 2);
            B = (int)((int)pixel[0] >> 2);

            kosong.at<int>(((B << 6)<<6) + (G << 6) + R) = nilai;
        }
    }
}


Mat GoalPostDetector::captureInCanvas(Mat frame, string yes){
    Mat hitam = Mat::zeros(frame.size(), CV_8UC3);

    for(int x=0; x< frame.cols; x++){
        for(int y=0; y<frame.rows; y++){
            Vec3b pixel = frame.at<Vec3b>(y,x);
            R = (int)((int)pixel[2] >> 2);
            G = (int)((int)pixel[1] >> 2);
            B = (int)((int)pixel[0] >> 2);
            if(yes == "hijau-putih"){
                switch(kosong.at<int>(((B << 6)<<6) + (G << 6) + R)){
                    case 1: hitam.at<Vec3b>(y,x) = Vec3b(0, 255, 0); break;
                    case 3: hitam.at<Vec3b>(y,x) = Vec3b(0, 0, 255); break;
                }
            }else if(yes == "hijau"){
                switch(kosong.at<int>(((B << 6)<<6) + (G << 6) + R)){
                    case 1: hitam.at<Vec3b>(y,x) = Vec3b(0, 255, 0); break;
                }
            }
        }
    }
    return hitam;
}


void GoalPostDetector::GetFrameCamCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        if(msg->encoding.compare(sensor_msgs::image_encodings::MONO8)==0){
            img_encoding_ = IMG_MONO;
        }else if(msg->encoding.compare(sensor_msgs::image_encodings::RGB8)==0){
            img_encoding_ = IMG_RGB;
        }
        cvImgPtr = cv_bridge::toCvCopy(msg,msg->encoding);

    }catch(cv_bridge::Exception &e){
        ROS_ERROR("%s",e.what());
    }
}


void GoalPostDetector::process(){
    if(cvImgPtr == nullptr)return;

    Mat f = cvImgPtr->image;
    cvtColor(f, frame, CV_RGB2BGR);

    namedWindow("frame", CV_WINDOW_NORMAL);
    imshow("frame", frame);
    
    Mat hsv;
    cvtColor(frame, hsv, CV_BGR2HSV);
    namedWindow("hsv", CV_WINDOW_NORMAL);
    imshow("hsv", hsv);

    Mat hs;
    vector<Mat> toSplit;
    split(hsv, toSplit);

    vector<Mat> tomerge;
    tomerge.push_back(toSplit[0]);
    tomerge.push_back(toSplit[1]);
    tomerge.push_back(Mat::zeros(frame.size(), CV_8UC1));
    toSplit.clear();

    merge(tomerge, hs);
    tomerge.clear();

    namedWindow("HS", CV_WINDOW_NORMAL);
    imshow("HS", hs);    

    Mat ycrcb;
    cvtColor(frame, ycrcb, CV_BGR2YCrCb);
    split(ycrcb, toSplit);

    tomerge.push_back(Mat::zeros(frame.size(), CV_8UC1));
    tomerge.push_back(toSplit[1]);
    tomerge.push_back(toSplit[2]);
    
    Mat y; merge(tomerge, y);


    namedWindow("crcb", CV_WINDOW_NORMAL);
    imshow("crcb", y);

    namedWindow("ycrcb", CV_WINDOW_NORMAL);
    imshow("ycrcb", ycrcb);

    Mat imY = toSplit[0];
    
    Sobel(imY, imYX,CV_8U, 1,0,3);
    Sobel(imY, imYY,CV_8U, 0,1,3);

    // namedWindow("outhsv", CV_WINDOW_NORMAL);
    // createTrackbar("hl", "outhsv", &hl, 179, treshSob);
    // createTrackbar("hh", "outhsv", &hh, 179, treshSob);

    // createTrackbar("sl", "outhsv", &sl, 179, treshSob);
    // createTrackbar("sh", "outhsv", &sh, 179, treshSob);

    // createTrackbar("vl", "outhsv", &vl, 179, treshSob);
    // createTrackbar("vh", "outhsv", &vh, 179, treshSob);

    // Mat outhsv;
    // inRange(frame, Scalar(hl, sl, vl), Scalar(hh, sh, vh), outhsv);
    // imshow("outhsv", outhsv);

    // imshow("imYX", imYX);
    // //imshow("imYY", imYY);


    Mat roiRGB = captureInCanvas(frame, "hijau");
    //namedWindow("roiRGB", CV_WINDOW_NORMAL);
    //imshow("roiRGB", roiRGB);

    Mat hijauPM = captureInCanvas(frame, "hijau-putih");
    namedWindow("hijauPM", CV_WINDOW_NORMAL);
    imshow("hijauPM", hijauPM);

    int key = waitKey(10);

    if((char)key == 'k'){
        namedWindow("kalibrasi frame", CV_WINDOW_NORMAL);
        setMouseCallback("kalibrasi frame", onMouse);
        int nilai = 0;

        gambar = frame;
        gambarClone = gambar.clone();

        while(true){
            int inkey = waitKey(10);
            imshow("kalibrasi frame", gambar);
            bool kalib = false;
            if((rectROI.width != 0) || (rectROI.height != 0) ){
                kalib = true;
                switch((char)inkey){
                case 'h':nilai= TABEL_HIJAU;//1;
                    masukanMatrix(nilai, gambar, rectROI);
                    cout << "saved hijau" << endl;
                    break;
                case 'o':nilai=TABEL_ORANGE;//2;
                    masukanMatrix(nilai, gambar, rectROI);
                    cout << "saved orange" << endl;
                    break;
                case 'p':nilai=TABEL_PUTIH; //3;
                    masukanMatrix(nilai, gambar, rectROI);
                    cout << "saved putih" << endl;
                    break;
                case 'd':nilai = TABEL_OTHER;//0;
                    masukanMatrix(nilai, gambar, rectROI);
                    cout << "saved other" << endl;
                    break;
                }
                if((char)inkey == 's')
                    simpanTabel(kosong, path_yaml.c_str());
            }
            if((char)inkey == 'c'){
                simpanTabel(kosong, path_yaml.c_str());
                destroyWindow("kalibrasi frame");
                if(kalib)
                    destroyWindow("roi");
                break;
              }
        }
    }
    
    Mat roiGray;
    cvtColor(roiRGB, roiGray, CV_BGR2GRAY);
    vector<vector<Point> >konturS;
    findContours(roiGray, konturS, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    if(konturS.size() == 0)return;

    vector<Point> konturT;

    for(size_t i=0; i< konturS.size(); i++){
        if(contourArea(konturS.at(i)) >= 200)
            konturT.insert(konturT.end(), konturS.at(i).begin(), konturS.at(i).end());
    }

    // Mat putih = Mat::ones(roiGray.size(), CV_8UC1);
    vector<Point> hull;
    if(konturT.size() != 0){
        convexHull(konturT, hull, false);
        // konturS.clear();
        // konturS.push_back(hull);
        // drawContours(putih, konturS,0,0,-1);
    }else return;

    // multiply(imYX, putih, imYX);
    // namedWindow("mulputih", CV_WINDOW_NORMAL);
    // imshow("mulputih", imYX);

    Mat fC = frame.clone();

   vector<Point> cbp;
   int stateb;
   int staten;

   bool awal = false;

   Point _masuk;

   Point HP, PH;
   
   

   // pengurangan nilai di setiap step -> ada yg nggak hijau

   vector<Point> hullClone = hull;
   
   for(int step=0; step<5; step++){
        int spaceHull = hull.size();
        if(step > 0){
            for(int s=0; s < spaceHull; s++){
                Point _hijau; bool masukHijau=false;
                Rect kotak((hull[s].x - 4)<0 ?0:(hull[s].x-4),
                            (hull[s].y - 4)<0 ?0:(hull[s].y-4),
                            (hull[s].x + 4)>639?(639-hull[s].x):8,
                            (hull[s].y + 4)>479?(479-hull[s].y):8);

                // rectangle(fC, kotak, Scalar(150,step*50,0),1);

                for(int r=kotak.tl().y; r<kotak.br().y; r++){
                    for(int c=kotak.tl().x; c<kotak.br().x; c++){
                        Vec3b pixel = frame.at<Vec3b>(r,c);

                        R = (int)((int)pixel[2] >> 2);
                        G = (int)((int)pixel[1] >> 2);
                        B = (int)((int)pixel[0] >> 2);
                        
                        int nilaiTabel = kosong.at<int>(((B << 6)<<6) + (G << 6) + R);
                        if(nilaiTabel == TABEL_HIJAU){
                            _hijau = Point(c,r);
                            masukHijau = true;
                        }
                    }
                }
                if(masukHijau)
                    hull.push_back(_hijau);
            }
            hull.erase(hull.begin(), hull.begin()+spaceHull-1);
        }

#define OK
#ifdef OK
        
       for(int s=0; s<spaceHull; s++){
           if(step == 0){
             if(hull[s].y == 479){
                 hull.erase(hull.begin()+s);
                 continue;
             }
           }

            if(!awal){
                awal = true;
                _masuk = hull[s];
            }else{
                float gdt = (hull[s].y - _masuk.y)/(hull[s].x - _masuk.x + 1e-6);

                for(int n_x=_masuk.x; ;){
                    int n_y = int(gdt * (n_x - _masuk.x) + _masuk.y);
                    // if(step == 2)
                    // circle(fC, Point(n_x, n_y), 1, Scalar(step*50,255-step*25,245), -1);

                    Vec3b pixel = frame.at<Vec3b>(n_y,n_x);

                    R = (int)((int)pixel[2] >> 2);
                    G = (int)((int)pixel[1] >> 2);
                    B = (int)((int)pixel[0] >> 2);
                    
                    int nilaiTabel = kosong.at<int>(((B << 6)<<6) + (G << 6) + R);
                        
                    if(s != 0 || n_x != _masuk.x)
                       stateb = staten;

                    switch(nilaiTabel){
                        case TABEL_HIJAU:
                            staten = HIJAU; break;
                        case TABEL_PUTIH:
                            staten = PUTIH; break;
                        default:
                            staten = HIJAU; break;
                    }

                    if( s==0){
                        if(n_x == _masuk.x){
                            stateb = staten;
                            
                            if(n_x < hull[s].x){
                                n_x ++;
                            }else if(n_x > hull[s].x){
                                n_x --;
                            }else{
                                break;
                            }
                            continue;
                        }
                    }
                    
                    if(stateb == HIJAU && staten == PUTIH){
                        HP = Point(n_x, n_y);
                        cbp.push_back(HP);
                    }
                    if(stateb == PUTIH && staten == HIJAU){
                        if(cbp.size() == 0) continue;
                        if((cbp[cbp.size()-1].x != HP.x) 
                                && (cbp[cbp.size()-1].y != HP.y))
                                {
                                    cbp.erase(cbp.end());
                                    continue;
                                }
                        PH = Point(n_x, n_y);
                        float panjang = sqrt(((PH-HP).x << 1) + ((PH-HP).y << 1));
                        if(panjang <= 10.0){
                            cbp.erase(cbp.end());
                            continue;
                        }
                        cbp.push_back(PH);
                        
                        circle(fC, HP, 4, Scalar(100,20,190), 1);
                        circle(fC, PH, 4, Scalar(100,20,190), 1);
                    }
                    if(n_x < hull[s].x){
                        n_x ++;
                    }else if(n_x > hull[s].x){
                        n_x --;
                    }else{
                        break;
                    }
                }
                _masuk = hull[s];
            }
        }

    double _min, _max;
    Point min_loc, max_loc;
    vector<Point> ujung;
    bool isi = false;

    for(size_t s=0; s<cbp.size(); s++){
        isi = false;
        Point titik__ = cbp[s];
        for(int rr=titik__.y; rr>0; rr--){
            Rect kotakGrad((titik__.x - 5)<0 ?0:(titik__.x-5),
                        (titik__.y - 2)<0 ?0:(titik__.y-2),
                        (titik__.x + 5)>639?(639-titik__.x):10,
                        (titik__.y - 1)>479?(479-titik__.y):2);
            
            // rectangle(fC, kotakGrad, Scalar(255,0,0), -1);
            
            Mat hitam(imYX, kotakGrad);
            minMaxLoc(hitam, &_min, &_max, &min_loc, &max_loc);

            Point gotoUjung = titik__ + max_loc;

            if(_max < uchar(int(0.5*(int)imYX.at<uchar>(titik__.y, titik__.x)))){
                isi = true;
                // cout << pointPolygonTest(hullClone, gotoUjung, false) << endl;
                if(pointPolygonTest(hullClone, gotoUjung, false) == +1 || pointPolygonTest(hullClone, gotoUjung, false) == 0)
                    break;
                ujung.push_back(gotoUjung);
                line(fC, cbp[s], gotoUjung, Scalar::all(0), 2);
                break;
            }else{
                // if(pointPolygonTest(hull, gotoUjung, false) == +1)
                //     break;
                titik__.y = titik__.y-kotakGrad.height+max_loc.y;
                titik__.x = titik__.x-0.5*kotakGrad.width+max_loc.x;
            }
        }
        if(!isi){
            if(pointPolygonTest(hullClone, titik__ + max_loc, false) == +1 || pointPolygonTest(hullClone, titik__ + max_loc, false) == 0)
                continue;
            ujung.push_back(titik__ + max_loc);
            // cout << pointPolygonTest(hullClone, titik__ + max_loc, false) << endl;
            line(fC, cbp[s], titik__ + max_loc, Scalar::all(0), 2);
        }
    }

    cbp.clear();
   }
    
#endif
   namedWindow("fC", CV_WINDOW_NORMAL);
   imshow("fC", fC);
   waitKey(1);
}

void treshSob(int, void*){
    // Mat hijau;
    // threshold(hsv, hijau, thrX, 255, CV_THRESH_BINARY);
    // imshow("hijau", hijau);
}

static void onMouse(int event, int x, int y, int, void*){
    int xrs, yrs, lx, ly;

    if(afterDownBeforeUp){
        gambar = gambarClone.clone();
        xrs = min(titikStart.x, x);
        yrs = min(titikStart.y, y);
        lx = max(titikStart.x, x) - min(titikStart.x, x);
        ly = max(titikStart.y, y) - min(titikStart.y, y);
        rectROI = Rect(xrs, yrs, lx+1, ly+1);

        rectangle(gambar, rectROI,Scalar(255, 0, 0), 1);
    }
    if(event == EVENT_LBUTTONDOWN){
        titikStart = Point(x,y);
        rectROI = Rect(x,y,0,0);
        afterDownBeforeUp = true;

    }else if(event == EVENT_LBUTTONUP){
        Mat roi(gambarClone.clone(), rectROI);
        imshow("roi", roi);

        afterDownBeforeUp = false;
    }
}
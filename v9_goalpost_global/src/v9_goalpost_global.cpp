#include <v9_goalpost_global/v9_goalpost_global.h>

Mat gambar;
Mat gambarClone;
Point titikStart;
bool afterDownBeforeUp = false;
Rect rectROI;


int hl = 0, sl = 0, vl = 0;
int hh = 179;
int sh = 255, vh = 255;


GoalPostDetector::GoalPostDetector()
                : nh_(ros::this_node::getName()),
                  it_(this->nh_)
{
    goal_pos_pub = nh_.advertise<geometry_msgs::Point>("goal_pos", 100);

    path_yaml = ros::package::getPath("v9_goalpost_global") + "/config/LookUpTable.yaml";
    
    get_frame_camera = it_.subscribe("/usb_cam_node/image_raw", 100, &GoalPostDetector::GetFrameCamCallback, this);

    canvas = bacaYAML(path_yaml.c_str());
    if(canvas.rows == 0)
        canvas = Mat::zeros(1, (64 << 12), CV_32SC1);
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

            canvas.at<int>(((B << 6)<<6) + (G << 6) + R) = nilai;
        }
    }
}


Mat GoalPostDetector::captureInCanvas(Mat _frame, int type_debug){
    Mat binHit = Mat::zeros(_frame.size(), CV_8UC1);

    for(int x=0; x< _frame.cols; x++){
        for(int y=0; y<_frame.rows; y++){
            Vec3b pixel = _frame.at<Vec3b>(y,x);
            R = (int)pixel[2] >> 2;
            G = (int)pixel[1] >> 2;
            B = (int)pixel[0] >> 2;

            int nilai_tabel = canvas.at<int>(((B << 6)<<6) + (G << 6) + R);

            switch(type_debug){
                case TYPE_HIJAU_BINER:
                    switch(nilai_tabel){
                        case TABEL_HIJAU: binHit.at<uchar>(y,x) = uchar(TABEL_HIJAU); break;
                    }break;
                case TYPE_HIJAU_PUTIH_BINER:
                    switch(nilai_tabel){
                        case TABEL_HIJAU: binHit.at<uchar>(y,x) = uchar(TABEL_HIJAU); break;
                        case TABEL_PUTIH: binHit.at<uchar>(y,x) = uchar(TABEL_PUTIH); break;
                    }break;
            }
        }        
    }
#ifndef DEBUG
    return binHit;
#elif defined(DEBUG)
    return binHit;
#endif
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

    Mat YCRCB;
    cvtColor(frame, YCRCB, CV_BGR2YCrCb);

    vector<Mat> YCC;
    split(YCRCB, YCC);

    Mat imY = YCC[0];
    
    Sobel(imY, imYX,CV_8U, 1,0,3);
    Sobel(imY, imYY,CV_8U, 0,1,3);

    namedWindow("imYX", CV_WINDOW_NORMAL);
    imshow("imYX", imYX);

    namedWindow("imYY", CV_WINDOW_NORMAL);
    imshow("imYY", imYY);

    segmentedGreen = captureInCanvas(frame, TYPE_HIJAU_BINER);
    segmentedGreenWhite = captureInCanvas(frame, TYPE_HIJAU_PUTIH_BINER);

#ifdef DEBUG
    // namedWindow("segmentedGreen", CV_WINDOW_NORMAL);
    // imshow("segmentedGreen", segmentedGreen);
    namedWindow("segmentedGreenWhite", CV_WINDOW_NORMAL);
    imshow("segmentedGreenWhite", 60*segmentedGreenWhite);
#endif


#ifdef KALIBRASI
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
                    simpanTabel(canvas, path_yaml.c_str());
            }
            if((char)inkey == 'c'){
                simpanTabel(canvas, path_yaml.c_str());
                destroyWindow("kalibrasi frame");
                if(kalib)
                    destroyWindow("roi");
                break;
              }
        }
    }
#endif
    
    vector<vector<Point> >konturS;
    findContours(segmentedGreen, konturS, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    if(konturS.size() == 0)return;

    vector<Point> konturT;

    for(size_t i=0; i< konturS.size(); i++){
        if(contourArea(konturS.at(i)) >= 200)
            konturT.insert(konturT.end(), konturS[i].begin(), konturS[i].end());
    }

    
    vector<Point> hull;
    if(konturT.size() != 0){
        convexHull(konturT, hull, false);
    }else return;

    Mat fC = frame.clone();

#ifdef PROCESS
   vector<LineIterator> LINEcbp;
   vector<vector<float > > diffScanLine;

   int stateb;
   int staten;

   Point _masuk;

   Point HP, PH;
   

   vector<Point> hullClone = hull;
   int maxstep = 7;
   
   for(int step=0; step<maxstep; step++){
        bool awal = false;
        int spaceHull = hull.size();
        konturS.clear();
        konturS.push_back(hull);

        Mat iteng = Mat::zeros(frame.size(), CV_8UC1);
        drawContours(iteng, konturS, 0, 1, CV_FILLED);
        erode(iteng, iteng, getStructuringElement(MORPH_RECT, Size(5,5)));
        findContours(iteng, konturS, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        konturT.clear();
        for(int s=0; s<konturS.size(); s++){
            konturT.insert(konturT.end(), konturS[s].begin(), konturS[s].end());
        }

        if(konturT.size() != 0){
            convexHull(konturT, hull);
        }else continue;

        if(step < 1) continue;
   
       for(int s=0; s<spaceHull; s++){
            if(hull[s].y >= 469){
                hull.erase(hull.begin()+s);
                continue;
            }

            if(!awal){
                awal = true;
                _masuk = hull[s];
            }else{
                float gdt = (hull[s].y - _masuk.y)/(hull[s].x - _masuk.x + 1e-6);
                int count = 0;

                LineIterator it(segmentedGreenWhite, _masuk, hull[s], 8);

                for(int i=0; i<it.count; i++, ++it){
                    int nilaiTabel = (int)segmentedGreenWhite.at<uchar>(it.pos());

                    // if(step == 5)
                    // circle(fC, it.pos(), 1, Scalar(step*50,255-step*25,245), -1);

                    int n_x = it.pos().x;
                        
                    if(s != 0 || n_x != _masuk.x)
                       stateb = staten;

                    switch(nilaiTabel){
                        case TABEL_HIJAU:
                            staten = STATE_HIJAU; break;
                        case TABEL_PUTIH:
                            staten = STATE_PUTIH; break;
                        default:
                            staten = STATE_HIJAU; break;
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
                    
                    if(stateb == STATE_HIJAU && staten == STATE_PUTIH){
                        HP = it.pos();
                        count ++;
                    }

                    if(count){
                        if(stateb == STATE_PUTIH && staten == STATE_PUTIH){
                            count ++;
                        }
                    }
                    if(stateb == STATE_PUTIH && staten == STATE_HIJAU){
                        if(count < 15){
                            count = 0;
                            continue;
                        }

                        count = 0;

                        PH = it.pos();

                        int x_HP = gdt*HP.y + HP.x; 
                        int y_HP = 0;
                        
                        if(x_HP > 639){
                            x_HP = 639;
                            y_HP = -1/gdt * (x_HP - HP.x) + HP.y;
                        }else if(x_HP < 0){
                            x_HP = 0;
                            y_HP = 1/gdt * HP.x + HP.y;
                        }

                        int x_PH = gdt*PH.y + PH.x;
                        int y_PH = 0;
                        if(x_PH > 639){
                            x_PH = 639;
                            y_PH = -1/gdt * (x_PH - PH.x) + PH.y;
                        }else if(x_PH < 0){
                            x_PH = 0;
                            y_PH = 1/gdt * PH.x + PH.y;
                        }

                        LineIterator itCbp(imYX, HP, Point(x_HP,y_HP), 8);

                        LINEcbp.push_back(itCbp);

                        itCbp = LineIterator(imYX, PH, Point(x_PH,y_PH), 8);

                        LINEcbp.push_back(itCbp);

                        line(fC, HP, Point(x_HP,y_HP), Scalar(255,0,0), 1);
                        line(fC, PH, Point(x_PH,y_PH), Scalar(0,0,255), 1);
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
   }

   if(LINEcbp.size() == 0) return;

    double _min, _max;
    Point min_loc, max_loc;
    vector<Point> ujung;
    bool isi = false;

    vector<float> v_vdp;
    float jml_vdp = 0.0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    for(int s=0; s < LINEcbp.size(); s++){
        LineIterator _lncbp = LINEcbp[s];
        vector<float> _diff;
        float value_diff_pth;


        for(int a=0; a<_lncbp.count; a++, ++_lncbp){
            Point _plcbp = _lncbp.pos();
            Rect _ktk((_plcbp.x-4)<0 ? 0:(_plcbp.x-4),
                      (_plcbp.y-4)<0 ? 0:(_plcbp.y-4),
                      (_plcbp.x+4)>639 ? 639-_plcbp.x:9,
                      (_plcbp.y+4)>479 ? 479-_plcbp.y:9);

            Mat crp(imYX, _ktk);
            Mat pth = 255*Mat::ones(crp.size(), CV_8UC1);
            pth = pth - crp;
            
            if(a>0){
                _diff.push_back(value_diff_pth - sum(pth)[0]);
            }
            value_diff_pth = sum(pth)[0];
            jml_vdp += value_diff_pth;
        }

        diffScanLine.push_back(_diff);
        v_vdp.push_back(jml_vdp);
    }

    // cout << "size v_vdp: " << v_vdp.size() << endl;
    // cout << "size LINEcbp: " << LINEcbp.size() << endl;

    quickSort(v_vdp, LINEcbp, diffScanLine, 0, v_vdp.size()-1);
    float _q2;
    calcQuartil(v_vdp, _q2, MEDIAN);

    for(int s=0; s< v_vdp.size(); s++){
        if(v_vdp[s] > _q2){
            v_vdp.erase(v_vdp.begin() + s);
            LINEcbp.erase(LINEcbp.begin() + s);
            diffScanLine.erase(diffScanLine.begin() + s);
        }
    }

    // cout << "after, LINEcbp size: " << LINEcbp.size() << endl;
    // cout << endl;

    std::vector< std::pair<Point, Point> > ujungFix;
    for(int s=0; s<LINEcbp.size(); s++){
        LineIterator _it = LINEcbp[s];
        Mat _mf_v_dsl = Mat(diffScanLine[s].size(), 2, CV_64F, diffScanLine[s].data());

        Point _minLoc, _maxLoc;
        double _minVal, _maxVal;

        minMaxLoc(_mf_v_dsl, &_minVal, &_maxVal, &_minLoc, &_maxLoc);

        std::pair<Point, Point> ujung;
        
        ujung.first = _it.pos();
        for(int c=0; c<_maxLoc.y; c++)_it++;

        // _it += _maxLoc.y;
        ujung.second = _it.pos();

        line(fC, ujung.first, ujung.second, Scalar::all(0), 2);

        ujungFix.push_back(ujung);
    }
#endif

   namedWindow("fC", CV_WINDOW_NORMAL);
   imshow("fC", fC);
   waitKey(1);
}

void treshSob(int, void*){

}

void SWWAP(float* a, float* b, LineIterator* vla, LineIterator* vlb, vector<float>* dfa, vector<float>* dfb)
{ 
    float t = *a; 
    *a = *b; 
    *b = t; 

    LineIterator vlt = *vla;
    *vla = *vlb;
    *vlb = vlt;

    vector<float> dft = *dfa;
    *dfa = *dfb;
    *dfb = dft;
} 
  
int partition (vector<float>& V, vector<LineIterator>& VL, vector<vector<float> >& DF, int low, int high) 
{ 
    int pivot = V[high];
    int i = (low - 1);
  
    for (int j = low; j <= high- 1; j++) 
    { 
        if (V[j] <= pivot) 
        { 
            i++;
            SWWAP(&V[i], &V[j], &VL[i], &VL[j], &DF[i], &DF[j]); 
        } 
    } 
    SWWAP(&V[i + 1], &V[high], &VL[i+1], &VL[high], &DF[i+1], &DF[high]); 
    return (i + 1); 
} 

void quickSort(vector<float>& V, vector<LineIterator>& VL, vector<vector<float> >& DF, int low, int high)
{ 
    if (low < high) 
    { 
        int pi = partition(V, VL, DF, low, high); 
        quickSort(V, VL, DF, low, pi - 1); 
        quickSort(V, VL, DF, pi + 1, high); 
    }
}

void calcQuartil(vector<float> V, float& quartil, int QQQ = MEDIAN){
    // if(!V.size()){
    //     cout << "Size V is 0" << endl;
    //     return;
    // }else{
    //     cout << "size V is " << V.size() << endl;
    // }
    switch(QQQ){
        // case Q1:{
        //     vector<float> _V;
        //     _V.insert(_V.end(), V.begin(), V.begin()+int(V.size()/2));
        //     calcQuartil(_V, quartil, MEDIAN);
        // }
        case MEDIAN:{
            if(V.size() % 2 != 0)
                quartil = V[V.size()/2+1];
            else
                quartil = 0.5*(V[V.size()/2] + V[V.size()/2+1]);
        }
        // case Q3:{
        //     vector<float> _V;
        //     _V.insert(_V.end(), V.begin()+V.size()/2, V.end());
        //     calcQuartil(_V, quartil, MEDIAN);
        // }
    }
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


/* LIST MASALAH:

1. color space
2. kontur mendalam -> TODO: erode [FIRST]
3. scan vertical -> TODO: along perpendicular
4. pembatasan tinggi
5. connect of point of convexhull
*/
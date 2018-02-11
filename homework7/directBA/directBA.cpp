//
// Created by xiang on 1/4/18.
// this program shows how to perform direct bundle adjustment
//
#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "./poses.txt";
string points_file = "./points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3();
    }

    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
//        cout << update.transpose() << endl;
        setEstimate(Sophus::SE3::exp(update) * estimate());
    }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double,16,1> Vector16d;
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectProjection(float *color, cv::Mat &target) {
        this->origColor = color;
        this->targetImg = target;
    }

    ~EdgeDirectProjection() {}

    virtual void computeError() override {
        // TODO START YOUR CODE HERE
        // compute projection error ...

        const g2o::VertexSBAPointXYZ* Point_vertex = static_cast<const g2o::VertexSBAPointXYZ*> (vertex(0));
        const VertexSophus* pose_vertex = static_cast<const VertexSophus*> (vertex(1));

        const g2o::Vector3D Point = Point_vertex->estimate();
        const Sophus::SE3 pose = pose_vertex -> estimate();

        g2o::Vector3D camera_Point = pose * Point;
        Eigen::Vector2d pixel;
        pixel(0) = fx * camera_Point(0) / camera_Point(2) + cx;
        pixel(1) = fy * camera_Point(1) / camera_Point(2) + cy;

        if (pixel(0) < 4 || pixel(0) > targetImg.cols - 4 || pixel(1) < 4 || pixel(1) > targetImg.rows - 4) {
            for (int k = 0;k < 16; k++) {
                _error(k, 0) = 0.0;
            }
            this -> setLevel(1);
        } else {
            int i = 0;
            for (int x = -2; x <= 1; x++) {
                for (int y= -2; y <= 1; y++) {

                    _error(i, 0) = origColor[i] - GetPixelValue(targetImg, pixel(0) + x, pixel(1) + y);
//                    if (vertex(1)->id() == 4119) {
//                        cout << "origColor[" << i << "]: " << origColor[i]
//                                << " pixel_x: " << pixel(0) + x
//                                << " pixel_y: " << pixel(1) + y
//                                << " GetPixelValue: "
//                             << GetPixelValue(targetImg, pixel(0) + x, pixel(1) + y)
//                             << " _error(i, 0): " << _error(i, 0) << endl;
//                    }

                    i++;
                }
            }
//            if (vertex(1)->id() == 4119) {
//                cout << vertex(1)->id()<<endl;
//                assert(false);
//            }


        }




        // END YOUR CODE HERE
    }

    // Let g2o compute jacobian for you

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

private:
    cv::Mat targetImg;  // the target image
    float *origColor = nullptr;   // 16 floats, the color of this point
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points);

int main(int argc, char **argv) {

    // read poses and points
    VecSE3 poses;
    VecVec3d points;
    ifstream fin(pose_file);

    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto &d: data) fin >> d;
        poses.push_back(Sophus::SE3(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        if (!fin.good()) break;
    }
    fin.close();


    vector<float *> color;
    fin.open(points_file);
    while (!fin.eof()) {
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) fin >> xyz[i];
        if (xyz[0] == 0) break;
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
        float *c = new float[16];
        for (int i = 0; i < 16; i++) fin >> c[i];
        color.push_back(c);

        if (fin.good() == false) break;
    }
    fin.close();

    cout << "poses: " << poses.size() << ", points: " << points.size() << endl;


    // read images
    vector<cv::Mat> images;
    boost::format fmt("./%d.png");
    for (int i = 0; i < 7; i++) {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }

    // build optimization problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock *solver_ptr = new DirectBlock(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // TODO add vertices, edges into the graph optimizer
    // START YOUR CODE HERE
    for (int i = 0; i < points.size(); i++) {
        g2o::VertexSBAPointXYZ* point_vertex = new g2o::VertexSBAPointXYZ();
        point_vertex -> setId(i);
        point_vertex -> setEstimate(points.at(i));
        point_vertex -> setMarginalized(true);
        optimizer.addVertex(point_vertex);
    }

    for (int i = 0; i < poses.size();i++) {
        VertexSophus* pose_vertex = new VertexSophus();
        pose_vertex -> setEstimate(poses.at(i));
        pose_vertex -> setId(points.size() + i);
        optimizer.addVertex(pose_vertex);
    }


    int edgeId = 1;
    for (int i = 0;i < poses.size();i++) {
        for (int j = 0;j < points.size();j++) {
//            Sophus::SE3 pose = poses.at(i);
//            Eigen::Vector3d point = points.at(j);
//
//            Eigen::Vector3d point_camera = pose * point;
//            Eigen::Vector2d pixel;
//            pixel(0) = fx * point_camera(0) / point_camera(2) + cx;
//            pixel(1) = fy * point_camera(1) / point_camera(2) + cy;
//
//            //　如果点不在该图片内，就不考虑了
//            if (pixel(0) < 20 || pixel(0) > images.at(i).cols - 20 || pixel(1) < 20 || pixel(1) > images.at(i).rows - 20) {
//                continue;
//            }

            // 在这里，就添加边
            EdgeDirectProjection* edge = new EdgeDirectProjection(color.at(j),images.at(i));

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            rk -> setDelta(0.5);
            edge -> setRobustKernel(rk);

            edge -> setId(edgeId++);
            edge -> setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(j)));
            edge -> setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(points.size() + i)));
            edge -> setInformation(Eigen::Matrix<double , 16, 16>::Identity());
            optimizer.addEdge(edge);
        }
    }
    // END YOUR CODE HERE

//     perform optimization
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // TODO fetch data from the optimizer
    // START YOUR CODE HERE
    for (int i = 0; i < points.size();i++) {
         points[i] = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i)) -> estimate();
    }

    for (int j = 0; j < poses.size();j++) {
        poses[j] = dynamic_cast<VertexSophus*>(optimizer.vertex(points.size() + j)) -> estimate();
    }
    // END YOUR CODE HERE

    // plot the optimized points and poses
    Draw(poses, points);

    // delete color data
    for (auto &c: color) delete[] c;
    return 0;
}

void Draw(const VecSE3 &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}


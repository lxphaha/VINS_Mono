/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-27 16:03:03
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-03-27 16:12:18
 * @FilePath: /catkin_vinsmono/src/VINS_Mono/camera_model/include/camodocal/chessboard/Chessboard.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CHESSBOARD_H
#define CHESSBOARD_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/calib3d/calib3d_c.h>

namespace camodocal
{

// forward declarations
class ChessboardCorner;
typedef boost::shared_ptr<ChessboardCorner> ChessboardCornerPtr;
class ChessboardQuad;
typedef boost::shared_ptr<ChessboardQuad> ChessboardQuadPtr;

class Chessboard
{
public:
    Chessboard(cv::Size boardSize, cv::Mat& image);

    void findCorners(bool useOpenCV = false);
    const std::vector<cv::Point2f>& getCorners(void) const;
    bool cornersFound(void) const;

    const cv::Mat& getImage(void) const;
    const cv::Mat& getSketch(void) const;

private:
    bool findChessboardCorners(const cv::Mat& image,
                               const cv::Size& patternSize,
                               std::vector<cv::Point2f>& corners,
                               int flags, bool useOpenCV);

    bool findChessboardCornersImproved(const cv::Mat& image,
                                       const cv::Size& patternSize,
                                       std::vector<cv::Point2f>& corners,
                                       int flags);

    void cleanFoundConnectedQuads(std::vector<ChessboardQuadPtr>& quadGroup, cv::Size patternSize);

    void findConnectedQuads(std::vector<ChessboardQuadPtr>& quads,
                            std::vector<ChessboardQuadPtr>& group,
                            int group_idx, int dilation);

//    int checkQuadGroup(std::vector<ChessboardQuadPtr>& quadGroup,
//                       std::vector<ChessboardCornerPtr>& outCorners,
//                       cv::Size patternSize);

    void labelQuadGroup(std::vector<ChessboardQuadPtr>& quad_group,
                        cv::Size patternSize, bool firstRun);

    void findQuadNeighbors(std::vector<ChessboardQuadPtr>& quads, int dilation);

    int augmentBestRun(std::vector<ChessboardQuadPtr>& candidateQuads, int candidateDilation,
                       std::vector<ChessboardQuadPtr>& existingQuads, int existingDilation);

    void generateQuads(std::vector<ChessboardQuadPtr>& quads,
                       cv::Mat& image, int flags,
                       int dilation, bool firstRun);

    bool checkQuadGroup(std::vector<ChessboardQuadPtr>& quads,
                        std::vector<ChessboardCornerPtr>& corners,
                        cv::Size patternSize);

    void getQuadrangleHypotheses(const std::vector< std::vector<cv::Point> >& contours,
                                 std::vector< std::pair<float, int> >& quads,
                                 int classId) const;

    bool checkChessboard(const cv::Mat& image, cv::Size patternSize) const;

    bool checkBoardMonotony(std::vector<ChessboardCornerPtr>& corners,
                            cv::Size patternSize);

    bool matchCorners(ChessboardQuadPtr& quad1, int corner1,
                      ChessboardQuadPtr& quad2, int corner2) const;

    cv::Mat mImage;
    cv::Mat mSketch;
    std::vector<cv::Point2f> mCorners;
    cv::Size mBoardSize;
    bool mCornersFound;
};

}

#endif

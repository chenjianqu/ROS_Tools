//
// Created by cjq on 23-8-14.
//

#ifndef COLMAP_POSE_TYPE_H
#define COLMAP_POSE_TYPE_H

#include "def.h"


template<class T>
class Pose {
public:
    Pose():q_(Eigen::Quaternion<T>::Identity()),t_(Eigen::Matrix<T,3,1>::Identity()){
    }
    Pose(const Eigen::Quaternion<T>& q,const Eigen::Matrix<T,3,1>& t):q_(q),t_(t){
        q_.normalize();
    }
    Pose(const Eigen::Matrix<T,3,3>& r,const Eigen::Matrix<T,3,1>& t):q_(r),t_(t){}

    explicit Pose(const Eigen::Matrix<T,4,4>& T4x4){
        Eigen::Matrix<T,3,3> R = T4x4.template topLeftCorner<3,3>();
        q_ = R;
        t_ = T4x4.template block<3,1>(0,3);
    }

    Pose(const Pose& pose){
        q_ = pose.q_;
        t_ = pose.t_;
    }

    Pose(Pose&& pose) noexcept {
        q_ = std::move(pose.q_);
        t_ = std::move(pose.t_);
    }

    Pose& operator= (const Pose& pose){
        q_ = pose.q_;
        t_ = pose.t_;
        return *this;
    }

    virtual ~Pose()= default;

    static constexpr double D2R = (M_PI / 180.0);
    static constexpr double R2D = (180.0 / M_PI);

    [[nodiscard]] Pose operator*(const Pose& other) const{
        return Pose<T>{q() * other.q(), q() * other.t() + t()};
    }

    template<class T1>
    [[nodiscard]] Pose<T1> cast(){
        return Pose<T1>{q_.template cast<T1>(),t_.template cast<T1>()};
    }

    static Pose Identity(){
        return Pose<T>{Eigen::Matrix<T,3,3>::Identity(),Eigen::Matrix<T,3,1>::Zero()};
    }

    [[nodiscard]] Pose Inverse() const{
        return Pose<T>{q_.inverse(), -(q_.inverse()*t_)};
    }

    [[nodiscard]] Vec3d operator*(const Vec3d& P) const{
        return q() * P + t();
    }

    [[nodiscard]] Eigen::Matrix<T,4,4> MakeMatrix4d() const {
        Eigen::Matrix<T,4,4> M = Eigen::Matrix<T,4,4> ::Zero();
        M(3, 3) = T(1);
        M.topLeftCorner(3,3) = q_.toRotationMatrix();
        M.topRightCorner(3,1) = t_;
        return M;
    }

    [[nodiscard]] Eigen::Matrix<T,7,1> MakeVec7d() const {
        Eigen::Matrix<T,7,1> ans;
        ans.topRows(3) = t();
        ans.bottomRows(4) = q().coeffs();
        return ans;
    }

    void SetVec7d(const Eigen::Matrix<T,7,1>& vec){
        t_ = vec.topRows(3);
        q_.coeffs() = vec.bottomRows(4);//x y z w
        q_.normalize();
    }

    [[nodiscard]] const Eigen::Quaternion<T> & q() const{
        return q_;
    }
    Eigen::Quaternion<T>& q(){
        return q_;
    }
    [[nodiscard]] const Eigen::Matrix<T,3,1> & t() const{
        return t_;
    }
    Eigen::Matrix<T,3,1>& t(){
        return t_;
    }

    [[nodiscard]] Eigen::Matrix<T,3,3> R() const{
        return q().toRotationMatrix();
    }

    [[nodiscard]] const Eigen::Matrix<double, 3, 1> euler() const{
        return R().eulerAngles(2,1,0);
    }

    [[nodiscard]] virtual std::string DebugString() const{
//            auto euler = this->euler();
//            auto t_d = t_;
//            return fmt::format("t:{:.3f} {:.3f} {:.3f} euler:{:.3f} {:.3f} {:.3f}",
//                               double(t_d.x()),double(t_d.y()),double(t_d.z()),
//                               euler.x(),euler.y(),euler.z());
        return "";
    }

protected:
    Eigen::Quaternion<T> q_;
    Eigen::Matrix<T,3,1> t_;
} ;


using Posed = Pose<double>;
using Posef = Pose<float>;


class PoseStamped : public Pose<double>{
public:
    PoseStamped() = default;
    PoseStamped(double time_stamp,const Quaterniond& q,const Vec3d &t):Pose(q,t),time_stamp_(time_stamp){}
    PoseStamped(double time_stamp,const Pose& pose):Pose(pose),time_stamp_(time_stamp){}
    explicit PoseStamped(double time_stamp,const Eigen::Matrix4d& T4x4):Pose<double>(T4x4),time_stamp_(time_stamp){}

    ~PoseStamped() override = default;

    static PoseStamped Identity(){
        return {0.,Eigen::Quaterniond ::Identity(),Eigen::Vector3d::Zero()};
    }

    static PoseStamped Identity(double time_stamp){
        return {time_stamp,Eigen::Quaterniond ::Identity(),Eigen::Vector3d::Zero()};
    }

    /**
     * 输入一个 T_left, 自身为T,计算 T_out = T_left * T
     * @param left_q
     * @param left_t
     * @return
     */
    [[nodiscard]] PoseStamped leftMul(const Quaterniond& left_q,const Vec3d &left_t) const{
        return {time_stamp_,left_q * q(), left_q * t() + left_t};
    }

    [[nodiscard]] PoseStamped leftMul(const Eigen::Matrix4d& left_T) const{
        Quaterniond left_q(left_T.topLeftCorner<3,3>());
        Vec3d left_t = left_T.block<3,1>(0,3);
        return {time_stamp_, left_q * q(), left_q * t() + left_t};
    }

    [[nodiscard]] PoseStamped leftMul(const Pose* left_T) const{
        return {time_stamp_,left_T->q() * q(), left_T->q() * t() + left_T->t()};
    }
    [[nodiscard]] PoseStamped leftMul(const Pose& left_T) const{
        return {time_stamp_,left_T.q() * q(), left_T.q() * t() + left_T.t()};
    }
    [[nodiscard]] PoseStamped leftMul(const PoseStamped& left_T) const{
        return {time_stamp_,left_T.q() * q(), left_T.q() * t() + left_T.t()};
    }

    [[nodiscard]] PoseStamped operator*(const Pose& other) const{
        return {time_stamp_,q() * other.q(), q() * other.t() + t()};
    }

    [[nodiscard]] PoseStamped operator*(const PoseStamped& other) const{
        return {time_stamp_,q() * other.q(), q() * other.t() + t()};
    }

    [[nodiscard]] Vec3d operator*(const Vec3d& P) const{
        return q() * P + t();
    }

    [[nodiscard]] PoseStamped Inverse() const{
        return {time_stamp_,Pose::Inverse()};
    }

    [[nodiscard]] std::string DebugString() const override  {
        auto euler = this->euler();
        auto t_d = t_;
        return fmt::format("timestamp:{} t:{:.3f} {:.3f} {:.3f} euler:{:.3f} {:.3f} {:.3f}",
                           time_stamp_,
                           double(t_d.x()),double(t_d.y()),double(t_d.z()),
                           euler.x(),euler.y(),euler.z());
    }

    [[nodiscard]] double timestamp() const{
        return time_stamp_;
    }

    double& timestamp(){
        return time_stamp_;
    }

private:
    double time_stamp_{};
};






#endif //COLMAP_POSE_TYPE_H

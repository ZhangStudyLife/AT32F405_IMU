/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
/* AT32 port: retained ellipsoid equations/quality gates; incremental solver,
 * native axes, explicit save, no old-board defaults or platform dependencies. */
#include "calibration.h"
#include <math.h>
#include <string.h>

#define MAX_POSES 128
CalParams cal_params;
CalStatus cal_status;
static float poses[MAX_POSES][3], mean[6], m2[6];
static float aug[9][10], solution[9], bias[3], matrix[3][3];
static unsigned solve_stage, row_index, pivot_index, eliminate_index;
static float residual_sum;
static float fabsf_local(float v) { return fabsf(v); }
static float norm3(const float *v) { return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
static int finite_value(float v) { return v == v && fabsf(v) < 1e6f; }
static void mat3_mul_vec(float m[3][3], const float v[3], float out[3])
{
    unsigned r;
    for (r=0; r<3; r++) out[r]=m[r][0]*v[0]+m[r][1]*v[1]+m[r][2]*v[2];
}
static int32_t mat3_inverse_sym(float A[3][3], float inv[3][3])
{
    float det;
    float cofactor[3][3];

    cofactor[0][0] = A[1][1] * A[2][2] - A[1][2] * A[2][1];
    cofactor[0][1] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]);
    cofactor[0][2] = A[1][0] * A[2][1] - A[1][1] * A[2][0];
    cofactor[1][0] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]);
    cofactor[1][1] = A[0][0] * A[2][2] - A[0][2] * A[2][0];
    cofactor[1][2] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]);
    cofactor[2][0] = A[0][1] * A[1][2] - A[0][2] * A[1][1];
    cofactor[2][1] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]);
    cofactor[2][2] = A[0][0] * A[1][1] - A[0][1] * A[1][0];

    det = A[0][0] * cofactor[0][0] + A[0][1] * cofactor[0][1] + A[0][2] * cofactor[0][2];
    if (fabsf_local(det) < 1.0e-12f)
    {
        return -1;
    }

    {
        float inv_det = 1.0f / det;
        uint8_t r, c;
        for (r = 0; r < 3; r++)
        {
            for (c = 0; c < 3; c++)
            {
                inv[r][c] = cofactor[c][r] * inv_det; /* transpose of cofactor */
            }
        }
    }
    return 0;
}

/* 3x3 Cholesky decomposition: A = L * L^T.
 * A must be symmetric positive-definite. L is lower-triangular output.
 * Returns 0 on success, -1 if not positive-definite. */
static int32_t cholesky_3x3(float A[3][3], float L[3][3])
{
    memset(L, 0, 9U * sizeof(float));

    if (A[0][0] <= 0.0f)
    {
        return -1;
    }
    L[0][0] = sqrtf(A[0][0]);
    L[1][0] = A[1][0] / L[0][0];
    L[2][0] = A[2][0] / L[0][0];

    {
        float v = A[1][1] - L[1][0] * L[1][0];
        if (v <= 0.0f)
        {
            return -1;
        }
        L[1][1] = sqrtf(v);
    }
    L[2][1] = (A[2][1] - L[2][0] * L[1][0]) / L[1][1];

    {
        float v = A[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1];
        if (v <= 0.0f)
        {
            return -1;
        }
        L[2][2] = sqrtf(v);
    }
    return 0;
}

static int busy(void)
{
    return cal_status.state >= CAL_READY && cal_status.state <= CAL_SOLVING;
}
static void fail(unsigned error)
{
    cal_status.error=error;
    cal_status.state=CAL_FAILED;
}
static void reset_window(void)
{
    cal_status.samples=cal_status.stable=0;
    memset(mean,0,sizeof(mean));
    memset(m2,0,sizeof(m2));
}
void Cal_Init(void)
{
    unsigned i;
    memset(&cal_params,0,sizeof(cal_params));
    memset(&cal_status,0,sizeof(cal_status));
    for(i=0;i<3;i++) cal_params.matrix[i][i]=1;
}
void Cal_Abort(void) { memset(&cal_status,0,sizeof(cal_status)); reset_window(); }
int Cal_StartGyro(void)
{
    if(busy()) return CAL_BUSY;
    Cal_Abort(); cal_status.mode=1; cal_status.state=CAL_WAIT;
    return CAL_OK;
}
int Cal_StartAccel(void)
{
    if(busy()) return CAL_BUSY;
    Cal_Abort(); cal_status.mode=2; cal_status.state=CAL_READY;
    return CAL_OK;
}
int Cal_Collect(void)
{
    if(cal_status.mode!=2 || cal_status.state!=CAL_READY) return CAL_BAD_COMMAND;
    if(cal_status.poses>=MAX_POSES) return CAL_BUSY;
    reset_window(); cal_status.state=CAL_WAIT;
    return CAL_OK;
}
static int add_pose(const float *v)
{
    unsigned i;
    float mag=norm3(v);
    if(mag<0.75f || mag>1.25f) return CAL_BAD_FIT;
    for(i=0;i<cal_status.poses;i++)
        if((poses[i][0]*v[0]+poses[i][1]*v[1]+poses[i][2]*v[2]) /
           (norm3(poses[i])*mag) > 0.965925826f) return CAL_DUPLICATE;
    memcpy(poses[cal_status.poses++],v,3*sizeof(float));
    return CAL_OK;
}
void Cal_Update(const float a[3], const float g[3])
{
    unsigned i;
    float gv[3], values[6];
    int stationary;
    if(!busy() || cal_status.state==CAL_SOLVING) return;
    if(++cal_status.total_samples>(cal_status.mode==1 ? 300000U:1800000U)) {
        fail(CAL_TIMEOUT); return;
    }
    if(cal_status.state==CAL_READY) return;
    for(i=0;i<3;i++) {
        if(!finite_value(a[i]) || !finite_value(g[i])) {
            if(cal_status.mode==2) { reset_window(); cal_status.state=CAL_WAIT; }
            return;
        }
        gv[i]=g[i]-cal_params.gyro_bias[i];
        values[i]=a[i]; values[i+3]=g[i];
    }
    stationary = cal_status.mode==1 ?
        (norm3(g)<6.0f && fabsf(norm3(a)-1.0f)<0.06f) :
        (norm3(gv)<3.0f && norm3(a)>=0.75f && norm3(a)<=1.25f);
    if(!stationary) {
        if(cal_status.mode==2 || cal_status.state==CAL_WAIT) {
            reset_window(); cal_status.state=CAL_WAIT;
        }
        cal_status.error=CAL_MOVING; return;
    }
    cal_status.error=CAL_OK;
    if(cal_status.state==CAL_WAIT) {
        if(++cal_status.stable < (cal_status.mode==1 ? 1500U:1250U)) return;
        cal_status.state=CAL_COLLECTING;
    }
    cal_status.samples++;
    for(i=0;i<6;i++) {
        float delta=values[i]-mean[i];
        mean[i]+=delta/(float)cal_status.samples;
        m2[i]+=delta*(values[i]-mean[i]);
    }
    if(cal_status.samples < (cal_status.mode==1 ? 60000U:5000U)) return;
    for(i=0;i<3;i++) {
        unsigned j=cal_status.mode==1 ? i+3:i;
        if(sqrtf(m2[j]/(float)(cal_status.samples-1)) >
           (cal_status.mode==1 ? 0.20f:0.025f)) {
            if(cal_status.mode==1) { fail(CAL_BAD_FIT); return; }
            reset_window(); cal_status.state=CAL_WAIT;
            cal_status.error=CAL_MOVING; return;
        }
    }
    if(cal_status.mode==1) {
        for(i=0;i<3;i++) if(fabsf(mean[i+3])>8) { fail(CAL_BAD_FIT); return; }
        memcpy(cal_params.gyro_bias,mean+3,3*sizeof(float));
        cal_params.flags|=1;
        cal_status.state=CAL_DONE;
    } else {
        cal_status.error=add_pose(mean);
        cal_status.state=CAL_READY;
    }
}
int Cal_Solve(void)
{
    if(cal_status.mode!=2 || cal_status.state!=CAL_READY) return CAL_BAD_COMMAND;
    if(cal_status.poses<12) { cal_status.error=CAL_TOO_FEW; return CAL_TOO_FEW; }
    memset(aug,0,sizeof(aug));
    memset(solution,0,sizeof(solution));
    solve_stage=row_index=pivot_index=0;
    cal_status.solve_steps=0;
    cal_status.fit_rms=cal_status.max_norm_error=residual_sum=0;
    cal_status.state=CAL_SOLVING;
    cal_status.error=CAL_OK;
    return CAL_OK;
}
/* Each call handles one pose, pivot, elimination row, or validation pose. */
void Cal_Step(void)
{
    unsigned r,c;
    if(cal_status.state!=CAL_SOLVING) return;
    cal_status.solve_steps++;
    if(solve_stage==0) {
        float x=poses[row_index][0], y=poses[row_index][1], z=poses[row_index][2];
        float v[9]={x*x,y*y,z*z,x*y,x*z,y*z,x,y,z};
        for(r=0;r<9;r++) {
            for(c=0;c<9;c++) aug[r][c]+=v[r]*v[c];
            aug[r][9]+=v[r];
        }
        if(++row_index==cal_status.poses) solve_stage=1;
    } else if(solve_stage==1) {
        unsigned best=pivot_index;
        for(r=pivot_index+1;r<9;r++)
            if(fabsf(aug[r][pivot_index])>fabsf(aug[best][pivot_index])) best=r;
        if(!finite_value(aug[best][pivot_index]) || fabsf(aug[best][pivot_index])<1e-12f) {
            fail(CAL_SINGULAR); return;
        }
        for(c=0;c<10;c++) {
            float tmp=aug[pivot_index][c];
            aug[pivot_index][c]=aug[best][c]; aug[best][c]=tmp;
        }
        eliminate_index=pivot_index+1; solve_stage=2;
    } else if(solve_stage==2) {
        if(eliminate_index<9) {
            float factor=aug[eliminate_index][pivot_index]/aug[pivot_index][pivot_index];
            for(c=pivot_index;c<10;c++) aug[eliminate_index][c]-=factor*aug[pivot_index][c];
            eliminate_index++;
        } else if(++pivot_index==9) { row_index=9; solve_stage=3; }
        else solve_stage=1;
    } else if(solve_stage==3) {
        float sum;
        r=--row_index; sum=aug[r][9];
        for(c=r+1;c<9;c++) sum-=aug[r][c]*solution[c];
        solution[r]=sum/aug[r][r];
        if(!row_index) solve_stage=4;
    } else if(solve_stage==4) {
        float q[3][3], inv[3][3], v[3], qb[3], lower[3][3], k;
        q[0][0]=solution[0]; q[1][1]=solution[1]; q[2][2]=solution[2];
        q[0][1]=q[1][0]=solution[3]*0.5f;
        q[0][2]=q[2][0]=solution[4]*0.5f;
        q[1][2]=q[2][1]=solution[5]*0.5f;
        for(r=0;r<3;r++) v[r]=-solution[r+6]*0.5f;
        if(mat3_inverse_sym(q,inv)) { fail(CAL_SINGULAR); return; }
        mat3_mul_vec(inv,v,bias); mat3_mul_vec(q,bias,qb);
        k=1+bias[0]*qb[0]+bias[1]*qb[1]+bias[2]*qb[2];
        if(!finite_value(k) || k<1e-6f || norm3(bias)>0.35f) { fail(CAL_BAD_FIT); return; }
        for(r=0;r<3;r++) for(c=0;c<3;c++) q[r][c]/=k;
        if(cholesky_3x3(q,lower)) { fail(CAL_BAD_FIT); return; }
        for(r=0;r<3;r++) for(c=0;c<3;c++) {
            matrix[r][c]=lower[c][r];
            if(!finite_value(matrix[r][c])) { fail(CAL_BAD_FIT); return; }
        }
        for(r=0;r<3;r++) if(matrix[r][r]<0.85f || matrix[r][r]>1.15f) {
            fail(CAL_BAD_FIT); return;
        }
        solve_stage=5; row_index=0;
    } else {
        float v[3], corrected[3], err;
        for(r=0;r<3;r++) v[r]=poses[row_index][r]-bias[r];
        mat3_mul_vec(matrix,v,corrected);
        err=fabsf(norm3(corrected)-1);
        if(!finite_value(err) || err>0.03f) { fail(CAL_BAD_FIT); return; }
        residual_sum+=err*err;
        if(err>cal_status.max_norm_error) cal_status.max_norm_error=err;
        if(++row_index==cal_status.poses) {
            cal_status.fit_rms=sqrtf(residual_sum/(float)cal_status.poses);
            memcpy(cal_params.accel_bias,bias,sizeof(bias));
            memcpy(cal_params.matrix,matrix,sizeof(matrix));
            cal_params.flags|=2;
            cal_status.state=CAL_DONE;
        }
    }
}
void Cal_Apply(const float a[3], const float g[3], float ao[3], float go[3])
{
    float v[3];
    unsigned i;
    for(i=0;i<3;i++) { v[i]=a[i]-cal_params.accel_bias[i]; go[i]=g[i]-cal_params.gyro_bias[i]; }
    mat3_mul_vec(cal_params.matrix,v,ao);
}
#ifdef IMU_TEST
int Cal_TestPose(const float a[3])
{
    if(cal_status.state!=CAL_READY || cal_status.poses>=MAX_POSES) return CAL_BAD_COMMAND;
    return add_pose(a);
}
#endif


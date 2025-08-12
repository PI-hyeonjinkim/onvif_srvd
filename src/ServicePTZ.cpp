/*
 --------------------------------------------------------------------------
 ServicePTZ.cpp

 Implementation of functions (methods) for the service:
 ONVIF ptz.wsdl server side
-----------------------------------------------------------------------------
*/

#include "soapPTZBindingService.h"
#include "ServiceContext.h"
#include "smacros.h"
#include "stools.h"

#include <string>
#include <map>
#include <mutex>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <algorithm>
#include <stdlib.h>
#include <unistd.h>

// ===== 장비 범위 (필요 시 조정) =====
static const float PAN_MIN  = 0.0f, PAN_MAX  = 180.0f;
static const float TILT_MIN = 0.0f, TILT_MAX = 180.0f;
static const float REL_SCALE_PAN  = 45.0f;  // 예: 0.5 → +22.5도
static const float REL_SCALE_TILT = 45.0f;
// ===== 프리셋 저장 위치 =====
static const char* PRESET_DB_DIR  = "/var/lib/onvif_srvd";
static const char* PRESET_DB_PATH = "/var/lib/onvif_srvd/presets.txt";

// ===== 유틸 =====
static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}


static void ensure_preset_dir() {
    struct stat st{};
    if (stat(PRESET_DB_DIR, &st) != 0) {
        mkdir(PRESET_DB_DIR, 0755);
    }
}

// run_system_cmd: 선언/정의 한 번만 (기본인자 정의부에만 두면 -fpermissive 회피)
static int run_system_cmd(const char* cmd, unsigned int timeout_usec = 0)
{
    int ret = system(cmd);
    DEBUG_MSG("PTZ cmd:%s  ret:%d\n", cmd, ret);
    if (timeout_usec) usleep(timeout_usec);
    return ret;
}
// ===== 현재 각도(누적) 상태 =====
static std::mutex g_pose_mtx;
static float g_cur_pan  = 0.0f;  // onvif_srvd 시작 시 카메라도 (0,0)이라 가정
static float g_cur_tilt = 0.0f;

static void set_current_pt(float pan, float tilt) {
    std::lock_guard<std::mutex> lk(g_pose_mtx);
    g_cur_pan  = clampf(pan,  PAN_MIN,  PAN_MAX);
    g_cur_tilt = clampf(tilt, TILT_MIN, TILT_MAX);
}

static void get_current_pt(float& pan, float& tilt) {
    std::lock_guard<std::mutex> lk(g_pose_mtx);
    pan = g_cur_pan; tilt = g_cur_tilt;
}

static void adjust_current_pt(float dpan, float dtilt) {
    std::lock_guard<std::mutex> lk(g_pose_mtx);
    g_cur_pan  = clampf(g_cur_pan  + dpan,  PAN_MIN,  PAN_MAX);
    g_cur_tilt = clampf(g_cur_tilt + dtilt, TILT_MIN, TILT_MAX);
}


static void goto_current_pt() {
    float pan, tilt;
    get_current_pt(pan, tilt);
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "curl -s http://127.0.0.1:7777/rotatePT/%.0f/%.0f", pan, tilt);
    run_system_cmd(cmd, 0);
    DEBUG_MSG("PTZ[MOVE]: goto pan=%.2f tilt=%.2f\n", pan, tilt);
}
// ===== 프리셋 저장소 =====
struct PresetRec {
    float pan, tilt, zoom;
    std::string name;
    PresetRec() : pan(0), tilt(0), zoom(1), name() {}
};

static std::mutex g_preset_mtx;
static std::map<std::string, PresetRec> g_presets; // token -> record

static void load_presets_from_file() {
    std::lock_guard<std::mutex> lock(g_preset_mtx);
    g_presets.clear();
    std::ifstream fin(PRESET_DB_PATH);
    if (!fin.is_open()) return;

    std::string token, name;
    float pan, tilt, zoom;
    // 한 줄 포맷: token pan tilt zoom name(공백X)
    while (fin >> token >> pan >> tilt >> zoom >> name) {
        PresetRec rec;
        rec.pan = pan; rec.tilt = tilt; rec.zoom = zoom; rec.name = name;
        g_presets[token] = rec;
    }
}

static void save_presets_to_file() {
    std::lock_guard<std::mutex> lock(g_preset_mtx);
    ensure_preset_dir();
    std::ofstream fout(PRESET_DB_PATH, std::ios::trunc);
    if (!fout.is_open()) return;

    for (std::map<std::string, PresetRec>::const_iterator it = g_presets.begin();
         it != g_presets.end(); ++it) {
        const std::string &t = it->first;
        const PresetRec  &r = it->second;
        std::string safe_name = r.name.empty() ? t : r.name; // 공백 없는 이름 권장
        fout << t << " " << r.pan << " " << r.tilt << " " << r.zoom << " " << safe_name << "\n";
    }
}

// ===== PTZ 노드/스페이스 (기존 유지, 프리셋 최대치만 확대) =====
static int GetPTZNode(struct soap *soap, tt__PTZNode* ptzn)
{
    if(!soap || !ptzn)
        return SOAP_FAULT;

    ptzn->token = "PTZNodeToken";
    ptzn->Name  = soap_new_std_string(soap, "PTZ");

    ptzn->SupportedPTZSpaces = soap_new_req_tt__PTZSpaces(soap);
    if(!ptzn->SupportedPTZSpaces)
        return SOAP_FAULT;

    soap_default_std__vectorTemplateOfPointerTott__Space2DDescription(soap, &ptzn->SupportedPTZSpaces->tt__PTZSpaces::RelativePanTiltTranslationSpace);
    soap_default_std__vectorTemplateOfPointerTott__Space1DDescription(soap, &ptzn->SupportedPTZSpaces->tt__PTZSpaces::RelativeZoomTranslationSpace);
    soap_default_std__vectorTemplateOfPointerTott__Space2DDescription(soap, &ptzn->SupportedPTZSpaces->tt__PTZSpaces::ContinuousPanTiltVelocitySpace);
    soap_default_std__vectorTemplateOfPointerTott__Space1DDescription(soap, &ptzn->SupportedPTZSpaces->tt__PTZSpaces::ContinuousZoomVelocitySpace);
    soap_default_std__vectorTemplateOfPointerTott__Space1DDescription(soap, &ptzn->SupportedPTZSpaces->tt__PTZSpaces::PanTiltSpeedSpace);
    soap_default_std__vectorTemplateOfPointerTott__Space1DDescription(soap, &ptzn->SupportedPTZSpaces->tt__PTZSpaces::ZoomSpeedSpace);

    tt__Space2DDescription* ptzs1 = soap_new_tt__Space2DDescription(soap);
    if(ptzs1) {
        ptzs1->URI         = "http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace";
        ptzs1->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
        ptzs1->YRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->RelativePanTiltTranslationSpace.emplace_back(ptzs1);

    tt__Space1DDescription* ptzs2 = soap_new_tt__Space1DDescription(soap);
    if(ptzs2) {
        ptzs2->URI         = "http://www.onvif.org/ver10/tptz/ZoomSpaces/TranslationGenericSpace";
        ptzs2->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->RelativeZoomTranslationSpace.emplace_back(ptzs2);

    tt__Space2DDescription* ptzs3 = soap_new_tt__Space2DDescription(soap);
    if(ptzs3) {
        ptzs3->URI         = "http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace";
        ptzs3->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
        ptzs3->YRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->ContinuousPanTiltVelocitySpace.emplace_back(ptzs3);

    tt__Space1DDescription* ptzs4 = soap_new_tt__Space1DDescription(soap);
    if(ptzs4) {
        ptzs4->URI         = "http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace";
        ptzs4->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->ContinuousZoomVelocitySpace.emplace_back(ptzs4);

    tt__Space1DDescription* ptzs5 = soap_new_tt__Space1DDescription(soap);
    if(ptzs5) {
        ptzs5->URI         = "http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace";
        ptzs5->XRange      = soap_new_req_tt__FloatRange(soap, 0.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->PanTiltSpeedSpace.emplace_back(ptzs5);

    tt__Space1DDescription* ptzs6 = soap_new_tt__Space1DDescription(soap);
    if(ptzs6) {
        ptzs6->URI         = "http://www.onvif.org/ver10/tptz/ZoomSpaces/ZoomGenericSpeedSpace";
        ptzs6->XRange      = soap_new_req_tt__FloatRange(soap, 0.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->ZoomSpeedSpace.emplace_back(ptzs6);

    ptzn->MaximumNumberOfPresets = 64; // 파일 기반이므로 충분히 크게
    ptzn->HomeSupported          = true;
    ptzn->FixedHomePosition      = soap_new_ptr(soap, true);

    return SOAP_OK;
}

int PTZBindingService::GetNodes(
    _tptz__GetNodes         *tptz__GetNodes,
    _tptz__GetNodesResponse &tptz__GetNodesResponse)
{
    UNUSED(tptz__GetNodes);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    soap_default_std__vectorTemplateOfPointerTott__PTZNode(
        soap, &tptz__GetNodesResponse._tptz__GetNodesResponse::PTZNode);

    tt__PTZNode* ptzn = soap_new_tt__PTZNode(soap);
    GetPTZNode(soap, ptzn);
    tptz__GetNodesResponse.PTZNode.emplace_back(ptzn);

    return SOAP_OK;
}

int PTZBindingService::GetNode(
    _tptz__GetNode         *tptz__GetNode,
    _tptz__GetNodeResponse &tptz__GetNodeResponse)
{
    UNUSED(tptz__GetNode);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    tptz__GetNodeResponse.PTZNode = soap_new_tt__PTZNode(soap);
    GetPTZNode(soap, tptz__GetNodeResponse.PTZNode);

    return SOAP_OK;
}

// ===== 프리셋 목록 =====
int PTZBindingService::GetPresets(
    _tptz__GetPresets         *tptz__GetPresets,
    _tptz__GetPresetsResponse &tptz__GetPresetsResponse)
{
    UNUSED(tptz__GetPresets);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    load_presets_from_file();

    soap_default_std__vectorTemplateOfPointerTott__PTZPreset(
        soap, &tptz__GetPresetsResponse._tptz__GetPresetsResponse::Preset);

    std::lock_guard<std::mutex> lock(g_preset_mtx);
    for (std::map<std::string, PresetRec>::const_iterator it = g_presets.begin();
         it != g_presets.end(); ++it) {
        const std::string &token = it->first;
        const PresetRec   &p     = it->second;

        tt__PTZPreset* ptzp = soap_new_tt__PTZPreset(soap);
        ptzp->token = soap_new_std_string(soap, token);
        ptzp->Name  = soap_new_std_string(soap, p.name.empty()? token : p.name);

        ptzp->PTZPosition = soap_new_req_tt__PTZVector(soap);
        ptzp->PTZPosition->PanTilt = soap_new_req_tt__Vector2D(soap, p.pan, p.tilt);
        ptzp->PTZPosition->Zoom    = soap_new_req_tt__Vector1D(soap, p.zoom);

        tptz__GetPresetsResponse.Preset.emplace_back(ptzp);
    }
    return SOAP_OK;
}

// ===== 프리셋 저장 (현재각 저장) =====
int PTZBindingService::SetPreset(
    _tptz__SetPreset         *tptz__SetPreset,
    _tptz__SetPresetResponse &tptz__SetPresetResponse)
{
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);
    if (!tptz__SetPreset || tptz__SetPreset->ProfileToken.empty())
        return SOAP_OK;

    load_presets_from_file();

    // 토큰 결정 (요청 없으면 자동 발급: 1,2,3,...)
    std::string token;
    if (tptz__SetPreset->PresetToken && !tptz__SetPreset->PresetToken->empty())
        token = *tptz__SetPreset->PresetToken;
    else {
        std::lock_guard<std::mutex> lock(g_preset_mtx);
        int next_id = 1;
        while (g_presets.count(std::to_string(next_id))) ++next_id;
        token = std::to_string(next_id);
    }

    // 현재각 저장
    PresetRec cur;
    get_current_pt(cur.pan, cur.tilt);
    cur.zoom = 1.0f;
    if (tptz__SetPreset->PresetName && !tptz__SetPreset->PresetName->empty())
        cur.name = *tptz__SetPreset->PresetName;

    {
        std::lock_guard<std::mutex> lock(g_preset_mtx);
        g_presets[token] = cur;
        save_presets_to_file();
    }

    tptz__SetPresetResponse.PresetToken = token;
    return SOAP_OK;
}

int PTZBindingService::RemovePreset(
    _tptz__RemovePreset         *tptz__RemovePreset,
    _tptz__RemovePresetResponse &tptz__RemovePresetResponse)
{
    UNUSED(tptz__RemovePresetResponse);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    if (!tptz__RemovePreset ||
        tptz__RemovePreset->ProfileToken.empty() ||
        tptz__RemovePreset->PresetToken.empty())
        return SOAP_OK;

    load_presets_from_file();
    {
        std::lock_guard<std::mutex> lock(g_preset_mtx);
        g_presets.erase(tptz__RemovePreset->PresetToken); // 값 타입 그대로 사용
        save_presets_to_file();
    }
    return SOAP_OK;
}
// ===== 프리셋 이동 =====
int PTZBindingService::GotoPreset(
    _tptz__GotoPreset         *tptz__GotoPreset,
    _tptz__GotoPresetResponse &tptz__GotoPresetResponse)
{
    UNUSED(tptz__GotoPresetResponse);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    if (!tptz__GotoPreset ||
        tptz__GotoPreset->ProfileToken.empty() ||
        tptz__GotoPreset->PresetToken.empty())
        return SOAP_OK;

    load_presets_from_file();
    PresetRec rec;
    {
        std::lock_guard<std::mutex> lock(g_preset_mtx);
        std::map<std::string, PresetRec>::const_iterator it =
            g_presets.find(tptz__GotoPreset->PresetToken); // 값 타입
        if (it == g_presets.end()) {
            DEBUG_MSG("PTZ: preset not found: %s\n",
                      tptz__GotoPreset->PresetToken.c_str());
            return SOAP_OK;
        }
        rec = it->second;
    }

    set_current_pt(rec.pan, rec.tilt);
    goto_current_pt();
    return SOAP_OK;
}
// ===== 홈 포지션 (0,0) =====
int PTZBindingService::GotoHomePosition(
    _tptz__GotoHomePosition         *req,
    _tptz__GotoHomePositionResponse &res)
{
    UNUSED(res);
    if (!req || req->ProfileToken.empty()) return SOAP_OK;
    set_current_pt(0.0f, 0.0f);
    goto_current_pt();
    return SOAP_OK;
}

// ===== AbsoluteMove (절대 이동) =====
int PTZBindingService::AbsoluteMove(
    _tptz__AbsoluteMove         *req,
    _tptz__AbsoluteMoveResponse &res)
{
    UNUSED(res);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    if (!req || req->ProfileToken.empty()) return SOAP_OK;
    if (!req->Position || !req->Position->PanTilt) return SOAP_OK;

    const float pan  = req->Position->PanTilt->x;
    const float tilt = req->Position->PanTilt->y;

    set_current_pt(pan, tilt);
    goto_current_pt();
    return SOAP_OK;
}

// ===== 사용 안 하는 핸들러는 비워두기 =====
int PTZBindingService::ContinuousMove(
    _tptz__ContinuousMove         *tptz__ContinuousMove,
    _tptz__ContinuousMoveResponse &tptz__ContinuousMoveResponse)
{
    UNUSED(tptz__ContinuousMove);
    UNUSED(tptz__ContinuousMoveResponse);
    DEBUG_MSG("PTZ: %s (unused)\n", __FUNCTION__);
    return SOAP_OK;
}

int PTZBindingService::RelativeMove(
    _tptz__RelativeMove         *req,
    _tptz__RelativeMoveResponse &res)
{
    UNUSED(res);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    if (!req || req->ProfileToken.empty()) return SOAP_OK;
    if (!req->Translation || !req->Translation->PanTilt) return SOAP_OK;

    const float rx = req->Translation->PanTilt->x;  // -1 ~ +1
    const float ry = req->Translation->PanTilt->y;

    const float dpan  = rx * REL_SCALE_PAN;
    const float dtilt = ry * REL_SCALE_TILT;

    if (dpan != 0.0f || dtilt != 0.0f) {
        adjust_current_pt(dpan, dtilt);  // 누적/클램프
        goto_current_pt();               // 절대 이동 호출
        DEBUG_MSG("PTZ[REL]: rx=%.3f ry=%.3f -> dpan=%.2f dtilt=%.2f\n", rx, ry, dpan, dtilt);
    }
    return SOAP_OK;
}
int PTZBindingService::Stop(_tptz__Stop *tptz__Stop, _tptz__StopResponse &tptz__StopResponse)
{
    UNUSED(tptz__Stop);
    UNUSED(tptz__StopResponse);
    DEBUG_MSG("PTZ: %s (noop)\n", __FUNCTION__);
    return SOAP_OK;
}

// ===== 나머지 비워두는 핸들러 =====
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetServiceCapabilities)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetConfigurations)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetStatus)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetConfiguration)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, SetConfiguration)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetConfigurationOptions)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, SetHomePosition)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, SendAuxiliaryCommand)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetPresetTours)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetPresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetPresetTourOptions)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, CreatePresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, ModifyPresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, OperatePresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, RemovePresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetCompatibleConfigurations)

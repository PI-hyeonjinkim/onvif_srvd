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
#include <iostream>
#include <fstream>

#define LOG_REQUEST(name) std::cout << "[PTZ] " << name << " called" << std::endl;

static int GetPTZPreset(struct soap *soap, tt__PTZPreset* ptzp, int number)
{
    ptzp->token = soap_new_std_string(soap, std::to_string(number));
    ptzp->Name  = soap_new_std_string(soap, std::to_string(number));


    ptzp->PTZPosition = soap_new_req_tt__PTZVector(soap);
    if(!ptzp->PTZPosition)
        return SOAP_FAULT;

    ptzp->PTZPosition->PanTilt = soap_new_req_tt__Vector2D(soap, 0.0f, 0.0f);
    ptzp->PTZPosition->Zoom    = soap_new_req_tt__Vector1D(soap, 1.0f);

    return SOAP_OK;
}



static int run_system_cmd(const char* cmd, unsigned int timeout_usec = 0)
{
    std::cout << "[CMD] system call: " << cmd << std::endl;
    int ret = system(cmd);

    if(timeout_usec)
        usleep(timeout_usec);

    return ret;
}



int PTZBindingService::GetPresets(
    _tptz__GetPresets         *tptz__GetPresets,
    _tptz__GetPresetsResponse &tptz__GetPresetsResponse)
{
    UNUSED(tptz__GetPresets);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);


    soap_default_std__vectorTemplateOfPointerTott__PTZPreset(
        soap, &tptz__GetPresetsResponse._tptz__GetPresetsResponse::Preset);

    for (int i = 0; i < 8; i++)
    {
        tt__PTZPreset* ptzp = soap_new_tt__PTZPreset(soap);

        if(!ptzp || GetPTZPreset(soap, ptzp, i))
            return SOAP_FAULT;

        tptz__GetPresetsResponse.Preset.emplace_back(ptzp);
    }

    return SOAP_OK;
}



int PTZBindingService::GotoPreset(
    _tptz__GotoPreset         *tptz__GotoPreset,
    _tptz__GotoPresetResponse &tptz__GotoPresetResponse)
{
    UNUSED(tptz__GotoPresetResponse);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);
    LOG_REQUEST("GotoPreset");

    std::string preset_cmd;

    auto ctx = (ServiceContext*)soap->user;

    if (!tptz__GotoPreset)
        return SOAP_OK;

    if (tptz__GotoPreset->ProfileToken.empty())
        return SOAP_OK;

    if (tptz__GotoPreset->PresetToken.empty())
        return SOAP_OK;


    if (!ctx->get_ptz_node()->get_move_preset().empty())
    {
        preset_cmd = ctx->get_ptz_node()->get_move_preset();
    } else
    {
        return SOAP_OK;
    }

    std::string template_str_t("%t");

    auto it_t = preset_cmd.find(template_str_t, 0);

    if( it_t != std::string::npos )
    {
        preset_cmd.replace(it_t, template_str_t.size(), tptz__GotoPreset->PresetToken.c_str());
    }

    run_system_cmd(preset_cmd.c_str());

    return SOAP_OK;
}



int GetPTZNode(struct soap *soap, tt__PTZNode* ptzn)
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


    auto ptzs1 = soap_new_tt__Space2DDescription(soap);
    if(ptzs1)
    {
        ptzs1->URI         = "http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace";
        ptzs1->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
        ptzs1->YRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->RelativePanTiltTranslationSpace.emplace_back(ptzs1);


    auto ptzs2 = soap_new_tt__Space1DDescription(soap);
    if(ptzs2)
    {
        ptzs2->URI         = "http://www.onvif.org/ver10/tptz/ZoomSpaces/TranslationGenericSpace";
        ptzs2->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->RelativeZoomTranslationSpace.emplace_back(ptzs2);


    auto ptzs3 = soap_new_tt__Space2DDescription(soap);
    if(ptzs3)
    {
        ptzs3->URI         = "http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace";
        ptzs3->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
        ptzs3->YRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->ContinuousPanTiltVelocitySpace.emplace_back(ptzs3);


    auto ptzs4 = soap_new_tt__Space1DDescription(soap);
    if(ptzs4)
    {
        ptzs4->URI         = "http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace";
        ptzs4->XRange      = soap_new_req_tt__FloatRange(soap, -1.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->ContinuousZoomVelocitySpace.emplace_back(ptzs4);


    auto ptzs5 = soap_new_tt__Space1DDescription(soap);
    if(ptzs5)
    {
        ptzs5->URI         = "http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace";
        ptzs5->XRange      = soap_new_req_tt__FloatRange(soap, 0.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->PanTiltSpeedSpace.emplace_back(ptzs5);


    auto ptzs6 = soap_new_tt__Space1DDescription(soap);
    if(ptzs6)
    {
        ptzs6->URI         = "http://www.onvif.org/ver10/tptz/ZoomSpaces/ZoomGenericSpeedSpace";
        ptzs6->XRange      = soap_new_req_tt__FloatRange(soap, 0.0f, 1.0f);
    }
    ptzn->SupportedPTZSpaces->ZoomSpeedSpace.emplace_back(ptzs6);


    ptzn->MaximumNumberOfPresets = 8;
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



int PTZBindingService::GotoHomePosition(
    _tptz__GotoHomePosition         *tptz__GotoHomePosition,
    _tptz__GotoHomePositionResponse &tptz__GotoHomePositionResponse)
{
    UNUSED(tptz__GotoHomePosition);
    UNUSED(tptz__GotoHomePositionResponse);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);
    LOG_REQUEST("GotoHomePosition");
    std::string preset_cmd;
	
    auto ctx = (ServiceContext*)soap->user;

    if (!tptz__GotoHomePosition)
        return SOAP_OK;

    if (tptz__GotoHomePosition->ProfileToken.empty())
        return SOAP_OK;


    if (!ctx->get_ptz_node()->get_move_preset().empty())
    {
        preset_cmd = ctx->get_ptz_node()->get_move_preset();
    }
    else
    {
        return SOAP_OK;
    }

    std::string template_str_t("%t");

    auto it_t = preset_cmd.find(template_str_t, 0);

    if( it_t != std::string::npos )
    {
        preset_cmd.replace(it_t, template_str_t.size(), "1");
    }

    run_system_cmd(preset_cmd.c_str());

    return SOAP_OK;
}



int PTZBindingService::ContinuousMove(
    _tptz__ContinuousMove         *tptz__ContinuousMove,
    _tptz__ContinuousMoveResponse &tptz__ContinuousMoveResponse)
{
    UNUSED(tptz__ContinuousMoveResponse);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    LOG_REQUEST("ContinuousMove");
    auto ctx = (ServiceContext*)soap->user;

    if (!tptz__ContinuousMove)
        return SOAP_OK;

    if (!tptz__ContinuousMove->Velocity)
        return SOAP_OK;

    if (!tptz__ContinuousMove->Velocity->PanTilt)
        return SOAP_OK;


    if (tptz__ContinuousMove->Velocity->PanTilt->x > 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_right().c_str());
    }
    else if (tptz__ContinuousMove->Velocity->PanTilt->x < 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_left().c_str());
    }

    if (tptz__ContinuousMove->Velocity->PanTilt->y > 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_up().c_str());
    }
    else if (tptz__ContinuousMove->Velocity->PanTilt->y < 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_down().c_str());
    }

    return SOAP_OK;
}



int PTZBindingService::RelativeMove(
    _tptz__RelativeMove         *tptz__RelativeMove,
    _tptz__RelativeMoveResponse &tptz__RelativeMoveResponse)
{
    UNUSED(tptz__RelativeMoveResponse);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);
    LOG_REQUEST("RelativeMove");


    auto ctx = (ServiceContext*)soap->user;

    if (!tptz__RelativeMove)
        return SOAP_OK;

    if (!tptz__RelativeMove->Translation)
        return SOAP_OK;

    if (!tptz__RelativeMove->Translation->PanTilt)
        return SOAP_OK;


    if (tptz__RelativeMove->Translation->PanTilt->x > 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_right().c_str(), 300000);
        run_system_cmd(ctx->get_ptz_node()->get_move_stop().c_str());
    }
    else if (tptz__RelativeMove->Translation->PanTilt->x < 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_left().c_str(), 300000);
        run_system_cmd(ctx->get_ptz_node()->get_move_stop().c_str());
    }

    if (tptz__RelativeMove->Translation->PanTilt->y > 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_up().c_str(), 300000);
        run_system_cmd(ctx->get_ptz_node()->get_move_stop().c_str());
    }
    else if (tptz__RelativeMove->Translation->PanTilt->y < 0)
    {
        run_system_cmd(ctx->get_ptz_node()->get_move_down().c_str(), 300000);
        run_system_cmd(ctx->get_ptz_node()->get_move_stop().c_str());
    }

    return SOAP_OK;
}



int PTZBindingService::Stop(_tptz__Stop *tptz__Stop, _tptz__StopResponse &tptz__StopResponse)
{
    UNUSED(tptz__Stop);
    UNUSED(tptz__StopResponse);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);
    LOG_REQUEST("Stop");

    auto ctx = (ServiceContext*)soap->user;

    run_system_cmd(ctx->get_ptz_node()->get_move_stop().c_str());

    return SOAP_OK;
}

int PTZBindingService::AbsoluteMove(_tptz__AbsoluteMove *req, _tptz__AbsoluteMoveResponse &res)
{
    float pan = 0.0f;
    float tilt = 0.0f;

    LOG_REQUEST("AbsoluteMove");

    if (req->Position && req->Position->PanTilt) {
        pan = req->Position->PanTilt->x * 90.0f;   // 예: 0.0 ~ 1.0 → 0~90도
        tilt = req->Position->PanTilt->y * 180.0f;
    }

    char cmd[256];
    // snprintf(cmd, sizeof(cmd), "curl -s http://127.0.0.1:7777/rotatePT/%.0f/%.0f", pan, tilt);
    // system(cmd);
    return SOAP_OK;
}

int PTZBindingService::GetConfigurations(
    _tptz__GetConfigurations* req,
    _tptz__GetConfigurationsResponse& res)
{
    UNUSED(req);
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);

    tt__PTZConfiguration* cfg = soap_new_tt__PTZConfiguration(soap);
    cfg->token = "PTZConfigToken";
    cfg->Name = soap_strdup(soap, "DefaultPTZConfig");
    cfg->NodeToken = "PTZNodeToken";
    cfg->UseCount = 1;

    res.PTZConfiguration.push_back(cfg);
    return SOAP_OK;
}

int PTZBindingService::GetConfiguration(
    _tptz__GetConfiguration* req,
    _tptz__GetConfigurationResponse& res)
{
    DEBUG_MSG("PTZ: %s\n", __FUNCTION__);
    UNUSED(req);

    res.PTZConfiguration = soap_new_tt__PTZConfiguration(soap);
    res.PTZConfiguration->token = "PTZConfigToken";
    res.PTZConfiguration->Name = soap_strdup(soap, "DefaultPTZConfig");
    res.PTZConfiguration->NodeToken = "PTZNodeToken";
    res.PTZConfiguration->UseCount = 1;

    return SOAP_OK;
}

int PTZBindingService::SetPreset(
    _tptz__SetPreset* req,
    _tptz__SetPresetResponse& res)
{
    LOG_REQUEST("SetPreset");

    if (!req) {
        std::cerr << "❌ SetPreset: req is null" << std::endl;
        return SOAP_FAULT;
    }

    if (!req->ProfileToken.empty())
        std::cout << "  ProfileToken: " << req->ProfileToken << std::endl;

    if (req->PresetName)
        std::cout << "  PresetName: " << *(req->PresetName) << std::endl;
    else
        std::cout << "  PresetName is null" << std::endl;

    if (req->PresetToken)
        std::cout << "  PresetToken: " << *(req->PresetToken) << std::endl;
    else
        std::cout << "  PresetToken is null" << std::endl;

    // PresetToken 필수로 가정하고 파일 저장
    if (req->PresetToken) {
        std::string filename = "/tmp/preset_" + *(req->PresetToken) + ".txt";
        std::ofstream file(filename);
        if (file.is_open()) {
            file << "PresetToken=" << *(req->PresetToken) << "\n";
            if (req->PresetName)
                file << "PresetName=" << *(req->PresetName) << "\n";
            file.close();
            std::cout << "  ✅ Preset saved to " << filename << std::endl;
        } else {
            std::cerr << "  ❌ Failed to write to " << filename << std::endl;
            return SOAP_FAULT;
        }

        // 응답용 PresetToken 설정
        res.PresetToken = soap_strdup(soap, req->PresetToken->c_str());
    } else {
        std::cerr << "  ❌ PresetToken is required to save preset" << std::endl;
        return SOAP_FAULT;
    }

    return SOAP_OK;
}

#include <fstream>  // 꼭 포함되어야 합니다

int PTZBindingService::SetHomePosition(
    _tptz__SetHomePosition* req,
    _tptz__SetHomePositionResponse& res)
{
    LOG_REQUEST("SetHomePosition");

    // 입력 로그 출력
    if (req && !req->ProfileToken.empty())
        std::cout << "  ProfileToken: " << req->ProfileToken << std::endl;
    else
        std::cout << "  ProfileToken is missing" << std::endl;

    // 예제 저장: 단순히 파일 생성
    const std::string filename = "/tmp/ptz_home.txt";
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "HomePreset=1" << std::endl;  // 임시 값. 실제 포지션 정보를 저장하려면 AbsoluteMove와 연동 필요
        file.close();
        std::cout << "  ✅ Home position saved to " << filename << std::endl;
    } else {
        std::cerr << "  ❌ Failed to write to " << filename << std::endl;
        return SOAP_FAULT;
    }

    return SOAP_OK;
}

SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetServiceCapabilities)
// SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetConfigurations)
// SOAP_EMPTY_HANDLER(PTZBindingService, tptz, SetPreset)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, RemovePreset)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetStatus)
// SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetConfiguration)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, SetConfiguration)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetConfigurationOptions)
// SOAP_EMPTY_HANDLER(PTZBindingService, tptz, SetHomePosition)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, SendAuxiliaryCommand)
//SOAP_EMPTY_HANDLER(PTZBindingService, tptz, AbsoluteMove)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetPresetTours)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetPresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetPresetTourOptions)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, CreatePresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, ModifyPresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, OperatePresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, RemovePresetTour)
SOAP_EMPTY_HANDLER(PTZBindingService, tptz, GetCompatibleConfigurations)
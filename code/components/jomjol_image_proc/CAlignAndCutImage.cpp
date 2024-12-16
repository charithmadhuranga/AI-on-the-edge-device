#include "CAlignAndCutImage.h"
#include "CRotateImage.h"
#include "ClassLogFile.h"

#include <math.h>
#include <algorithm>
#include <esp_log.h>
#include "psram.h"
#include "../../include/defines.h"

static const char* TAG = "c_align_and_cut_image";

CAlignAndCutImage::CAlignAndCutImage(std::string _name, CImageBasis *_org, CImageBasis *_temp) : CImageBasis(_name)
{
    name = _name;
    rgb_image = _org->rgb_image;
    channels = _org->channels;
    width = _org->width;
    height = _org->height;
    bpp = _org->bpp;
    externalImage = true;   

    islocked = false; 

    ImageTMP = _temp;
}

void CAlignAndCutImage::GetRefSize(int *ref_dx, int *ref_dy)
{
    ref_dx[0] = t0_dx;
    ref_dy[0] = t0_dy;
    ref_dx[1] = t1_dx;
    ref_dy[1] = t1_dy;
}

int CAlignAndCutImage::Align(RefInfo *_temp1, RefInfo *_temp2)
{
    CFindTemplate *ft = new CFindTemplate("align", rgb_image, channels, width, height, bpp);

    //////////////////////////////////////////////
    int r1_x = _temp1->target_x;
    int r1_y = _temp1->target_y;

    ESP_LOGD(TAG, "Before ft->FindTemplate(_temp1); %s", _temp1->image_file.c_str());
    bool isSimilar1 = ft->FindTemplate(_temp1);

    _temp1->width = ft->tpl_width;
    _temp1->height = ft->tpl_height;

    int dx1 = _temp1->target_x - _temp1->found_x;
    int dy1 = _temp1->target_y - _temp1->found_y;

    r1_x += dx1;
    r1_y += dy1;

    //////////////////////////////////////////////
    int r2_x = _temp2->target_x;
    int r2_y = _temp2->target_y;

    ESP_LOGD(TAG, "Before ft->FindTemplate(_temp2); %s", _temp2->image_file.c_str());
    bool isSimilar2 = ft->FindTemplate(_temp2);

    _temp2->width = ft->tpl_width;
    _temp2->height = ft->tpl_height;

    int dx2 = _temp2->target_x - _temp2->found_x;
    int dy2 = _temp2->target_y - _temp2->found_y;

    r2_x += dx1;
    r2_y += dy1;

    delete ft;

    //////////////////////////////////////////////
    float angle_org = atan2(_temp2->found_y - _temp1->found_y, _temp2->found_x - _temp1->found_x);
    float angle_cur = atan2(r2_y - r1_y, r2_x - r1_x);

    float angle_dif = (angle_cur - angle_org) * 180 / M_PI;

    if ((fabs(angle_dif) > (_temp1->search_max_angle || _temp2->search_max_angle)) || (abs(dx1) >= _temp1->search_x) || (abs(dy1) >= _temp1->search_y) || (abs(dx2) >= _temp2->search_x) || (abs(dy2) >= _temp2->search_y))
    {
        ESP_LOGD(TAG, "Alignment failed: dx1 %d - dy1 %d - dx2 %d - dy2 %d - rot %f", dx1, dy1, dx2, dy2, angle_dif);
        return -1; // ALIGNMENT FAILED
    }

    CRotateImage rt("Align", this, ImageTMP);

    if ((dx1 > 0 && dx2 > 0 && dy1 > 0 && dy2 > 0) || (dx1 < 0 && dx2 < 0 && dy1 < 0 && dy2 < 0))
    {
        // only move linaer because no rotative motion obviuos
        ESP_LOGD(TAG, "Align: Correction by linear dx + dy only");
        rt.Translate(dx1, dy1);
    }
    else if ((dx1 > 0 && dx2 > 0) || (dx1 < 0 && dx2 < 0))
    {
        // only rotate + move x direction
        ESP_LOGD(TAG, "Align: Correction by rotation + linear dx");
        rt.Rotate(angle_dif, width / 2, height / 2);
        rt.Translate(dx1 / 2, 0); // correct only by half because some correction already happen with rotation
    }
    else if ((dy1 > 0 && dy2 > 0) || (dy1 < 0 && dy2 < 0))
    {
        // only rotate + move y direction
        ESP_LOGD(TAG, "Align: Correction by rotation + linear dy");
        rt.Rotate(angle_dif, width / 2, height / 2);
        rt.Translate(0, dy1 / 2); // correct only by half because some correction already happen with rotation
    }
    else
    {
        // only rotate because no obviuos linear motion detected
        ESP_LOGD(TAG, "Align: Correction by rotation only");
        rt.Rotate(angle_dif, width / 2, height / 2);
    }

    ESP_LOGD(TAG, "Alignment: dx1 %d - dy1 %d - dx2 %d - dy2 %d - rot %f", dx1, dy1, dx2, dy2, angle_dif);

    if (isSimilar1 && isSimilar2)
    {
        return 1; // ALGO FAST match
    }
    else
    {
        return 0; // ALGO STANDARD done
    }
}

void CAlignAndCutImage::CutAndSave(std::string _template1, int x1, int y1, int dx, int dy)
{
    int x2, y2;

    x2 = x1 + dx;
    y2 = y1 + dy;
    x2 = std::min(x2, width - 1);
    y2 = std::min(y2, height - 1);

    dx = x2 - x1;
    dy = y2 - y1;

    int memsize = dx * dy * channels;
    uint8_t* odata = (unsigned char*) malloc_psram_heap(std::string(TAG) + "->odata", memsize, MALLOC_CAP_SPIRAM);

    stbi_uc* p_target;
    stbi_uc* p_source;

    RGBImageLock();

    for (int x = x1; x < x2; ++x)
    {
        for (int y = y1; y < y2; ++y)
        {
            p_target = odata + (channels * ((y - y1) * dx + (x - x1)));
            p_source = rgb_image + (channels * (y * width + x));
            for (int _channels = 0; _channels < channels; ++_channels)
            {
                p_target[_channels] = p_source[_channels];
            }
        }
    }

#ifdef STBI_ONLY_JPEG
    stbi_write_jpg(_template1.c_str(), dx, dy, channels, odata, 100);
#else
    stbi_write_bmp(_template1.c_str(), dx, dy, channels, odata);
#endif
    

    RGBImageRelease();

    stbi_image_free(odata);
}

void CAlignAndCutImage::CutAndSave(int x1, int y1, int dx, int dy, CImageBasis *_target)
{
    int x2, y2;

    x2 = x1 + dx;
    y2 = y1 + dy;
    x2 = std::min(x2, width - 1);
    y2 = std::min(y2, height - 1);

    dx = x2 - x1;
    dy = y2 - y1;

    if ((_target->height != dy) || (_target->width != dx) || (_target->channels != channels))
    {
        ESP_LOGD(TAG, "CAlignAndCutImage::CutAndSave - Image size does not match!");
        return;
    }

    uint8_t* odata = _target->RGBImageLock();
    RGBImageLock();

    stbi_uc* p_target;
    stbi_uc* p_source;

    for (int x = x1; x < x2; ++x)
    {
        for (int y = y1; y < y2; ++y)
        {
            p_target = odata + (channels * ((y - y1) * dx + (x - x1)));
            p_source = rgb_image + (channels * (y * width + x));
            for (int _channels = 0; _channels < channels; ++_channels)
            {
                p_target[_channels] = p_source[_channels];
            }
        }
    }

    RGBImageRelease();
    _target->RGBImageRelease();
}

CImageBasis* CAlignAndCutImage::CutAndSave(int x1, int y1, int dx, int dy)
{
    int x2, y2;

    x2 = x1 + dx;
    y2 = y1 + dy;
    x2 = std::min(x2, width - 1);
    y2 = std::min(y2, height - 1);

    dx = x2 - x1;
    dy = y2 - y1;

    int memsize = dx * dy * channels;
    uint8_t* odata = (unsigned char*)malloc_psram_heap(std::string(TAG) + "->odata", memsize, MALLOC_CAP_SPIRAM);

    stbi_uc* p_target;
    stbi_uc* p_source;

    RGBImageLock();

    for (int x = x1; x < x2; ++x)
    {
        for (int y = y1; y < y2; ++y)
        {
            p_target = odata + (channels * ((y - y1) * dx + (x - x1)));
            p_source = rgb_image + (channels * (y * width + x));
            for (int _channels = 0; _channels < channels; ++_channels)
            {
                p_target[_channels] = p_source[_channels];
            }
        }
    }

    CImageBasis* rs = new CImageBasis("CutAndSave", odata, channels, dx, dy, bpp);
    RGBImageRelease();
    rs->SetIndepended();
    return rs;
}

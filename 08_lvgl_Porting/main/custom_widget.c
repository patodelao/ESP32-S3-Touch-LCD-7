#include "lvgl.h"

static lv_style_t style_title;
static lv_style_t style_text_muted;

void create_custom_widget(lv_obj_t * parent) {
    /* Inicializar estilos */
    lv_style_init(&style_title);
    lv_style_set_text_font(&style_title, LV_FONT_DEFAULT);

    lv_style_init(&style_text_muted);
    lv_style_set_text_opa(&style_text_muted, LV_OPA_50);

    /* Crear el panel principal */
    lv_obj_t * panel1 = lv_obj_create(parent);
    lv_obj_set_height(panel1, LV_SIZE_CONTENT);
    lv_obj_set_width(panel1, lv_pct(100));
    lv_obj_center(panel1);

    /* Elementos dentro del panel */
    LV_IMG_DECLARE(img_demo_widgets_avatar);
    lv_obj_t * avatar = lv_img_create(panel1);
    lv_img_set_src(avatar, &img_demo_widgets_avatar);

    lv_obj_t * name = lv_label_create(panel1);
    lv_label_set_text(name, "Elena Smith");
    lv_obj_add_style(name, &style_title, 0);

    lv_obj_t * title = lv_label_create(panel1);
    lv_label_set_text(title, "Your profile");
    lv_obj_add_style(title, &style_title, 0);

    lv_obj_t * user_name_label = lv_label_create(panel1);
    lv_label_set_text(user_name_label, "User name");
    lv_obj_add_style(user_name_label, &style_text_muted, 0);

    lv_obj_t * user_name = lv_textarea_create(panel1);
    lv_textarea_set_one_line(user_name, true);
    lv_textarea_set_placeholder_text(user_name, "Your name");

    lv_obj_t * password_label = lv_label_create(panel1);
    lv_label_set_text(password_label, "Password");
    lv_obj_add_style(password_label, &style_text_muted, 0);

    lv_obj_t * password = lv_textarea_create(panel1);
    lv_textarea_set_one_line(password, true);
    lv_textarea_set_password_mode(password, true);
    lv_textarea_set_placeholder_text(password, "Min. 8 chars.");

    /* Posiciones */
    lv_obj_align(avatar, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_align(name, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 150);
    lv_obj_align(user_name_label, LV_ALIGN_TOP_LEFT, 20, 200);
    lv_obj_align(user_name, LV_ALIGN_TOP_LEFT, 20, 230);
    lv_obj_align(password_label, LV_ALIGN_TOP_LEFT, 20, 270);
    lv_obj_align(password, LV_ALIGN_TOP_LEFT, 20, 300);
}

# SPDX-License-Identifier: GPL-2.0
dtb-y += exynos/exynos3830.dtb
dtbo-y += samsung/a21s/a21s_kor_single_00.dtbo
dtbo-y += samsung/a21s/a21s_kor_single_01.dtbo
dtbo-y += samsung/a21s/a21s_kor_single_02.dtbo
dtbo-y += samsung/a21s/a21s_kor_single_03.dtbo
dtbo-y += samsung/a21s/a21s_kor_single_06.dtbo

targets += dtbs
DTB_LIST  := $(dtb-y) $(dtbo-y)
always    := $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := *.dtb*

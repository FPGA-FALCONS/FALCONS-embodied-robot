// 避障导航 UI 的行为/显示编码；无路为运行中的显示状态。
localparam [2:0] OBS_BROWSE = 3'd0,
                 OBS_EDIT   = 3'd1,
                 OBS_RUN    = 3'd2,
                 OBS_PAUSE  = 3'd3,
                 OBS_DONE   = 3'd4,
                 OBS_FAULT  = 3'd5,
                 OBS_NO_PATH = 3'd6;

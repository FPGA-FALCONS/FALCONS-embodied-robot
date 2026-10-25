// 八方向闭线段逐格枚举，斜向额外输出每步的两个侧格，禁止穿角。
// start 锁存两端；坐标差、几何判定分拍。valid 等 ready，last 伴随末格。
module nav_line_cells (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        start,
    input  wire        cancel,
    input  wire [15:0] x0, y0, x1, y1,
    output wire        valid,
    input  wire        ready,
    output reg  [15:0] x, y,
    output wire        last,
    output reg         invalid
);
    localparam [1:0] IDLE=0, GEOMETRY=1, RUN=2;
    reg [1:0] state, phase;
    reg [15:0] cx, cy, ex, ey, ax, ay;
    reg signed [1:0] sx, sy;
    assign valid = reset_n && state==RUN;
    assign last = phase==0 && cx==ex && cy==ey;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            phase <= 0;
            cx <= 0;
            cy <= 0;
            ex <= 0;
            ey <= 0;
            ax <= 0;
            ay <= 0;
            sx <= 0;
            sy <= 0;
            x <= 0;
            y <= 0;
            invalid <= 0;
        end else if (cancel) begin
            state <= IDLE;
            invalid <= 0;
        end else if (start) begin
            cx <= x0;
            cy <= y0;
            x <= x0;
            y <= y0;
            ex <= x1;
            ey <= y1;
            ax <= x1>=x0 ? x1-x0 : x0-x1;
            ay <= y1>=y0 ? y1-y0 : y0-y1;
            sx <= x1==x0 ? 0 : x1>x0 ? 2'sd1 : -2'sd1;
            sy <= y1==y0 ? 0 : y1>y0 ? 2'sd1 : -2'sd1;
            phase <= 0;
            invalid <= 0;
            state <= GEOMETRY;
        end else case (state)
            GEOMETRY: begin
                invalid <= ax!=0 && ay!=0 && ax!=ay;
                state <= ax!=0 && ay!=0 && ax!=ay ? IDLE : RUN;
            end
            RUN: if (ready) begin
                if (last) state <= IDLE;
                else if (sx!=0 && sy!=0 && phase==0) begin
                    x <= cx+{{14{sx[1]}},sx};
                    phase <= 1;
                end else if (phase==1) begin
                    x <= cx;
                    y <= cy+{{14{sy[1]}},sy};
                    phase <= 2;
                end else begin
                    cx <= cx+{{14{sx[1]}},sx};
                    cy <= cy+{{14{sy[1]}},sy};
                    x <= cx+{{14{sx[1]}},sx};
                    y <= cy+{{14{sy[1]}},sy};
                    phase <= 0;
                end
            end
            default: state <= IDLE;
        endcase
    end
endmodule

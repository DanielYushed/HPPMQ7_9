module data_unpacker (
    input  wire clk,
    input  wire rst_n,

    input  wire rx_ready,
    input  wire [7:0] rx_data,

    output reg  [15:0] sys_nc,
    output reg  [15:0] sys_rr,
    output reg  [15:0] sys_m1,
    output reg  [15:0] sys_m2,
    output reg  [15:0] sys_maxsteps,
    output reg  [15:0] sys_a0,
    output reg  [15:0] sys_b0,
    output reg  [15:0] sys_a1,
    output reg  [15:0] sys_b1,
    
    output reg         ram_we,       
    output reg  [9:0]  ram_addr,     
    output reg  [15:0] ram_data_out,

    output reg         config_done
);

    localparam WAIT_H1   = 3'd0;
    localparam WAIT_H2   = 3'd1;
    localparam GLOBALS   = 3'd2;
    localparam OBSTACLES = 3'd3;
    localparam DONE      = 3'd4;

    reg [2:0] state;

    reg        byte_toggle; 
    reg [7:0]  high_byte;
    reg [3:0]  global_cnt;
    reg [15:0] obs_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= WAIT_H1;
            byte_toggle  <= 0;
            global_cnt   <= 0;
            obs_cnt      <= 0;
            ram_we       <= 0;
            ram_addr     <= 0;
            config_done  <= 0;

            sys_nc <= 0; sys_rr <= 0; sys_m1 <= 0; sys_m2 <= 0;
            sys_maxsteps <= 0; sys_a0 <= 0; sys_b0 <= 0; sys_a1 <= 0; sys_b1 <= 0;
        end else begin
            ram_we <= 1'b0;

            if (ram_we == 1'b1) begin
                ram_addr <= ram_addr + 1'b1;
            end

            if (rx_ready) begin
                case (state)
                    WAIT_H1: begin
                        config_done <= 1'b0;
                        if (rx_data == 8'hAA) state <= WAIT_H2;
                    end
                    
                    WAIT_H2: begin
                        if (rx_data == 8'h55) begin
                            state       <= GLOBALS;
                            byte_toggle <= 1'b0;
                            global_cnt  <= 4'd0;
                        end else if (rx_data == 8'hAA) begin
                            state <= WAIT_H2;
                        end else begin
                            state <= WAIT_H1;
                        end
                    end
                    
                    GLOBALS: begin
                        if (byte_toggle == 1'b0) begin
                            high_byte   <= rx_data;
                            byte_toggle <= 1'b1;
                        end else begin
                            byte_toggle <= 1'b0;
                            case (global_cnt)
                                4'd0: sys_nc       <= {high_byte, rx_data};
                                4'd1: sys_rr       <= {high_byte, rx_data};
                                4'd2: sys_m1       <= {high_byte, rx_data};
                                4'd3: sys_m2       <= {high_byte, rx_data};
                                4'd4: sys_maxsteps <= {high_byte, rx_data};
                                4'd5: sys_a0       <= {high_byte, rx_data};
                                4'd6: sys_b0       <= {high_byte, rx_data};
                                4'd7: sys_a1       <= {high_byte, rx_data};
                                4'd8: begin 
                                      sys_b1   <= {high_byte, rx_data};
                                      state    <= OBSTACLES;
                                      obs_cnt  <= 16'd0;
                                      ram_addr <= 10'd0;
                                end
                            endcase
                            global_cnt <= global_cnt + 1'b1;
                        end
                    end
                    
                    OBSTACLES: begin
                        if (sys_nc == 0) begin
                            config_done <= 1'b1;
                            state       <= DONE;
                        end else if (byte_toggle == 1'b0) begin
                            high_byte   <= rx_data;
                            byte_toggle <= 1'b1;
                        end else begin
                            byte_toggle  <= 1'b0;
                            ram_data_out <= {high_byte, rx_data};
                            ram_we       <= 1'b1;
                            
                            if (obs_cnt == (sys_nc * 4) - 1) begin
                                config_done <= 1'b1;
                                state       <= DONE;
                            end else begin
                                obs_cnt  <= obs_cnt + 1'b1;
                            end
                        end
                    end
                    
                    DONE: begin
                        config_done <= 1'b1; 
                        
                        if (rx_data == 8'hAA) begin
                            state       <= WAIT_H2;
                            config_done <= 1'b0;
                            byte_toggle <= 1'b0;
                            global_cnt  <= 4'd0;
                            obs_cnt     <= 16'd0;
                            ram_addr    <= 10'd0;
                        end
                    end
                    
                    default: begin
                        state <= WAIT_H1;
                        config_done <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule 
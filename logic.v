`timescale 1ns / 1ps

// game_logic.v
// Contains frame_tick and ship_ctrl for moving the ship.

//////////////////////////////////////////////////////////////
// frame_tick: generates a slow enable pulse for game logic
//////////////////////////////////////////////////////////////
module frame_tick #(
    parameter integer TICK_HZ = 50        // how many ticks per second
)(
    input  wire clk_pix,                  // 25 MHz pixel clock
    input  wire reset,
    output reg  tick_out
);
    // 25,000,000 / 50 = 500,000 cycles per tick
    localparam integer CLK_FREQ   = 25000000;
    localparam integer TICK_COUNT = CLK_FREQ / TICK_HZ;

    reg [31:0] cnt;

    always @(posedge clk_pix or posedge reset) begin
        if (reset) begin
            cnt      <= 0;
            tick_out <= 1'b0;
        end else begin
            if (cnt == TICK_COUNT - 1) begin
                cnt      <= 0;
                tick_out <= 1'b1;   // one-cycle pulse
            end else begin
                cnt      <= cnt + 1;
                tick_out <= 1'b0;
            end
        end
    end
endmodule


//////////////////////////////////////////////////////////////
// ship_ctrl: keeps track of ship position and moves it
//////////////////////////////////////////////////////////////
module ship_ctrl (
    input  wire       clk_pix,
    input  wire       reset,
    input  wire       game_tick,     // from frame_tick
    input  wire       btn_left,
    input  wire       btn_right,

    output reg [9:0]  ship_x,
    output reg [9:0]  ship_y
);
    localparam integer H_VISIBLE = 640;
    localparam integer SHIP_W    = 32;
    localparam integer SHIP_H    = 16;
    localparam integer SHIP_SPEED = 4;

    // Vertical position: near bottom of screen
    localparam integer SHIP_Y_POS = 480 - 40;  // 40 pixels from bottom

    wire move_left  = btn_left  & ~btn_right;
    wire move_right = btn_right & ~btn_left;

    always @(posedge clk_pix or posedge reset) begin
        if (reset) begin
            // Start roughly centered
            ship_x <= (H_VISIBLE - SHIP_W) / 2;
            ship_y <= SHIP_Y_POS[9:0];
        end else if (game_tick) begin
            // Update only on game_tick (slow)
            if (move_left) begin
                if (ship_x > SHIP_SPEED)
                    ship_x <= ship_x - SHIP_SPEED;
                else
                    ship_x <= 0;
            end else if (move_right) begin
                if (ship_x < H_VISIBLE - SHIP_W - SHIP_SPEED)
                    ship_x <= ship_x + SHIP_SPEED;
                else
                    ship_x <= H_VISIBLE - SHIP_W;
            end
        end
    end

endmodule


//////////////////////////////////////////////////////////////
// bullet_mgr: one auto-firing bullet that moves upward
//////////////////////////////////////////////////////////////
module bullet_mgr (
    input  wire       clk_pix,
    input  wire       reset,
    input  wire       game_tick,     // same tick as ship
    input  wire [9:0] ship_x,
    input  wire [9:0] ship_y,
    input  wire       bullet_clear,  // NEW: clear when collision happens

    output reg        bullet_active,
    output reg [9:0]  bullet_x,
    output reg [9:0]  bullet_y
);
    localparam integer BULLET_W = 4;
    localparam integer BULLET_H = 8;
    localparam integer BULLET_SPEED = 6;   // pixels per tick

    // How often to auto-fire (in ticks)
    localparam integer FIRE_INTERVAL_TICKS = 30;  // adjust to taste

    reg [15:0] fire_cnt;

    always @(posedge clk_pix or posedge reset) begin
        if (reset) begin
            bullet_active <= 1'b0;
            bullet_x      <= 0;
            bullet_y      <= 0;
            fire_cnt      <= 0;
        end else if (game_tick) begin
            // Clear bullet immediately on hit
            if (bullet_active && bullet_clear) begin
                bullet_active <= 1'b0;
            end else if (!bullet_active) begin
                // No bullet active: count toward next shot
                if (fire_cnt >= FIRE_INTERVAL_TICKS) begin
                    fire_cnt      <= 0;
                    bullet_active <= 1'b1;
                    // Spawn bullet at top-center of ship
                    bullet_x      <= ship_x + 16 - (BULLET_W/2);  // ship width = 32
                    bullet_y      <= ship_y - BULLET_H;
                end else begin
                    fire_cnt <= fire_cnt + 1;
                end
            end else begin
                // Bullet is active: move it upward
                if (bullet_y > BULLET_SPEED) begin
                    bullet_y <= bullet_y - BULLET_SPEED;
                end else begin
                    // Off the top of the screen -> deactivate
                    bullet_active <= 1'b0;
                end
            end
        end
    end

endmodule


//////////////////////////////////////////////////////////////
// enemy_mgr: formation of enemies (rows x cols) moving like
// Space Invaders. Handles:
//  - side-to-side + drop movement
//  - collision with player bullet
//  - game_over_formation when formation too low
//  - you_win when all enemies are dead
//////////////////////////////////////////////////////////////
module enemy_mgr #(
    parameter integer N_ROWS    = 3,
    parameter integer N_COLS    = 8,
    parameter integer N_ENEMIES = N_ROWS * N_COLS
)(
    input  wire        clk_pix,
    input  wire        reset,
    input  wire        game_tick,

    // Player bullet info
    input  wire        bullet_active,
    input  wire [9:0]  bullet_x,
    input  wire [9:0]  bullet_y,

    output reg  [9:0]  group_x,         // leftmost x of formation
    output reg  [9:0]  group_y,         // top y of formation
    output reg  [N_ENEMIES-1:0] alive_mask,
    output reg         bullet_hit,      // 1 tick when bullet hits
    output reg         game_over_formation, // formation reached too low
    output reg         you_win          // all enemies destroyed
);
    localparam integer H_VISIBLE   = 640;
    localparam integer V_VISIBLE   = 480;

    localparam integer ENEMY_W     = 24;
    localparam integer ENEMY_H     = 16;
    localparam integer H_SPACING   = 16;   // horizontal spacing
    localparam integer V_SPACING   = 12;   // vertical spacing

    localparam integer START_X     = 60;
    localparam integer START_Y     = 60;
    localparam integer DROP_STEP   = 20;   // drop when bouncing

    localparam integer SPEED       = 2;    // horizontal pixels per tick

    localparam integer BULLET_W    = 4;
    localparam integer BULLET_H    = 8;

    // Width & height of full formation bounding box
    localparam integer FORMATION_W =
        N_COLS*ENEMY_W + (N_COLS-1)*H_SPACING;
    localparam integer FORMATION_H =
        N_ROWS*ENEMY_H + (N_ROWS-1)*V_SPACING;

    // threshold for game_over (when bottom of formation passes this)
    localparam integer GAME_OVER_Y = V_VISIBLE - 80;

    // direction flag: 1 = moving right, 0 = moving left
    reg dir_right;

    // count remaining enemies
    reg [7:0] enemy_count;   // enough for up to 255

    integer r, c, idx;
    integer ex0, ex1, ey0, ey1;
    integer bx0, bx1, by0, by1;
    integer bottom_y;

    always @(posedge clk_pix or posedge reset) begin
        if (reset) begin
            group_x            <= START_X[9:0];
            group_y            <= START_Y[9:0];
            dir_right          <= 1'b1;
            alive_mask         <= {N_ENEMIES{1'b1}};
            bullet_hit         <= 1'b0;
            game_over_formation<= 1'b0;
            you_win            <= 1'b0;
            enemy_count        <= N_ENEMIES[7:0];
        end else if (game_tick && !game_over_formation && !you_win) begin
            // default: no hit this tick
            bullet_hit <= 1'b0;

            // --- movement: horizontal until edge, then drop ---
            if (dir_right) begin
                if (group_x + FORMATION_W + SPEED < H_VISIBLE) begin
                    group_x <= group_x + SPEED;
                end else begin
                    dir_right <= 1'b0;
                    // drop down when hitting right edge
                    if (group_y + FORMATION_H + DROP_STEP < V_VISIBLE)
                        group_y <= group_y + DROP_STEP;
                end
            end else begin
                if (group_x > SPEED) begin
                    group_x <= group_x - SPEED;
                end else begin
                    dir_right <= 1'b1;
                    // drop down when hitting left edge
                    if (group_y + FORMATION_H + DROP_STEP < V_VISIBLE)
                        group_y <= group_y + DROP_STEP;
                end
            end

            // --- game over check: bottom of formation too low ---
            bottom_y = group_y + FORMATION_H;
            if (bottom_y >= GAME_OVER_Y)
                game_over_formation <= 1'b1;

            // --- bullet collision check (player bullet vs enemies) ---
            if (bullet_active) begin
                bx0 = bullet_x;
                bx1 = bullet_x + BULLET_W;
                by0 = bullet_y;
                by1 = bullet_y + BULLET_H;

                for (r = 0; r < N_ROWS; r = r+1) begin
                    for (c = 0; c < N_COLS; c = c+1) begin
                        idx = r*N_COLS + c;
                        if (alive_mask[idx]) begin
                            ex0 = group_x + c * (ENEMY_W + H_SPACING);
                            ex1 = ex0 + ENEMY_W;
                            ey0 = group_y + r * (ENEMY_H + V_SPACING);
                            ey1 = ey0 + ENEMY_H;

                            if ((bx1 > ex0) && (bx0 < ex1) &&
                                (by1 > ey0) && (by0 < ey1)) begin
                                alive_mask[idx] <= 1'b0;   // enemy dies
                                bullet_hit       <= 1'b1;  // tell bullet_mgr to clear

                                // decrement remaining enemies
                                if (enemy_count > 0)
                                    enemy_count <= enemy_count - 1;

                                // if we are killing the last one -> YOU WIN
                                if (enemy_count == 8'd1)
                                    you_win <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule


//////////////////////////////////////////////////////////////
// enemy_bullet_mgr: one enemy bullet moving downward.
// Picks a random column, then uses the bottom-most alive
// invader in that column as the shooter.
//////////////////////////////////////////////////////////////
module enemy_bullet_mgr #(
    parameter integer N_ROWS    = 3,
    parameter integer N_COLS    = 8,
    parameter integer N_ENEMIES = N_ROWS * N_COLS
)(
    input  wire        clk_pix,
    input  wire        reset,
    input  wire        game_tick,
    input  wire        game_over,

    input  wire [9:0]  group_x,      // formation top-left
    input  wire [9:0]  group_y,
    input  wire [N_ENEMIES-1:0] alive_mask,

    output reg         ebullet_active,
    output reg  [9:0]  ebullet_x,
    output reg  [9:0]  ebullet_y
);
    localparam integer ENEMY_W      = 24;
    localparam integer ENEMY_H      = 16;
    localparam integer H_SPACING    = 16;
    localparam integer V_SPACING    = 12;

    localparam integer BULLET_W     = 4;
    localparam integer BULLET_H     = 8;
    localparam integer BULLET_SPEED = 4;
    localparam integer V_VISIBLE    = 480;

    // fire every N ticks (roughly)
    localparam integer FIRE_INTERVAL_TICKS = 60;

    reg [15:0] fire_cnt;

    // Simple 8-bit LFSR for "random" value
    reg  [7:0] lfsr;
    wire [7:0] rand_val = lfsr;

    // helpers for picking shooter
    reg [2:0] rand_col;          // 0..7
    integer   r, idx;
    reg [1:0] shooter_row;       // up to 3 rows (0..2)
    reg       found_in_column;

    // spawn position temps
    integer sx;
    integer sy;

    // LFSR update
    always @(posedge clk_pix or posedge reset) begin
        if (reset) begin
            lfsr <= 8'h3C;
        end else begin
            // x^8 + x^6 + x^5 + x^4 + 1 taps
            lfsr <= {lfsr[6:0],
                     lfsr[7] ^ lfsr[5] ^ lfsr[4] ^ lfsr[3]};
        end
    end

    // Enemy bullet logic
    always @(posedge clk_pix or posedge reset) begin
        if (reset) begin
            ebullet_active <= 1'b0;
            ebullet_x      <= 0;
            ebullet_y      <= 0;
            fire_cnt       <= 0;
        end else if (game_tick && !game_over) begin
            if (!ebullet_active) begin
                // count toward next enemy shot
                if (fire_cnt >= FIRE_INTERVAL_TICKS) begin
                    fire_cnt <= 0;

                    // pick a random column 0..7 from LFSR
                    rand_col = rand_val[2:0];  // 3 bits

                    // find bottom-most alive in that column
                    found_in_column = 1'b0;
                    shooter_row     = 0;

                    for (r = 0; r < N_ROWS; r = r+1) begin
                        idx = r*N_COLS + rand_col;
                        if (alive_mask[idx]) begin
                            // last alive we see (largest r) is bottom-most
                            found_in_column = 1'b1;
                            shooter_row     = r[1:0];
                        end
                    end

                    if (found_in_column) begin
                        // spawn from that invader's bottom-center
                        sx = group_x +
                             rand_col * (ENEMY_W + H_SPACING) +
                             (ENEMY_W/2 - BULLET_W/2);
                        sy = group_y +
                             shooter_row * (ENEMY_H + V_SPACING) +
                             ENEMY_H;  // bottom of invader

                        ebullet_active <= 1'b1;
                        ebullet_x      <= sx[9:0];
                        ebullet_y      <= sy[9:0];
                    end else begin
                        // no alive in this column -> skip shot this time
                        ebullet_active <= 1'b0;
                    end
                end else begin
                    fire_cnt <= fire_cnt + 1;
                end
            end else begin
                // move bullet downward
                if (ebullet_y + BULLET_SPEED < V_VISIBLE) begin
                    ebullet_y <= ebullet_y + BULLET_SPEED;
                end else begin
                    // off bottom of screen
                    ebullet_active <= 1'b0;
                end
            end
        end
    end

endmodule



//////////////////////////////////////////////////////////////
// ship_hit: detects when enemy bullet hits the ship.
// Latches ship_dead = 1 until reset.
//////////////////////////////////////////////////////////////
module ship_hit (
    input  wire       clk_pix,
    input  wire       reset,
    input  wire       game_tick,       // can use game_tick (not masked)
    input  wire       ebullet_active,
    input  wire [9:0] ebullet_x,
    input  wire [9:0] ebullet_y,
    input  wire [9:0] ship_x,
    input  wire [9:0] ship_y,

    output reg        ship_dead
);
    localparam integer SHIP_W    = 32;
    localparam integer SHIP_H    = 16;
    localparam integer BULLET_W  = 4;
    localparam integer BULLET_H  = 8;

    integer sx0, sx1, sy0, sy1;
    integer bx0, bx1, by0, by1;

    always @(posedge clk_pix or posedge reset) begin
        if (reset) begin
            ship_dead <= 1'b0;
        end else if (!ship_dead && game_tick) begin
            // Ship rectangle
            sx0 = ship_x;
            sx1 = ship_x + SHIP_W;
            sy0 = ship_y;
            sy1 = ship_y + SHIP_H;

            // Enemy bullet rectangle
            bx0 = ebullet_x;
            bx1 = ebullet_x + BULLET_W;
            by0 = ebullet_y;
            by1 = ebullet_y + BULLET_H;

            if (ebullet_active &&
                (bx1 > sx0) && (bx0 < sx1) &&
                (by1 > sy0) && (by0 < sy1)) begin
                ship_dead <= 1'b1;   // GAME OVER for player
            end
        end
    end

endmodule

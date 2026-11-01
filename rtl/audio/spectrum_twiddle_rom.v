`timescale 1ns/1ps
`default_nettype none
// FFT 正余弦系数 ROM，共 512 个 18 位有符号定点数，小数位数为 17。
// addr[8]=0 选择余弦，=1 选择正弦；addr[7:0] 为系数索引。
// 在 clk 上升沿读取地址，同步输出结果，数据不复位。
// USE_EG_BRAM=1 使用器件 BRAM，=0 使用行为查表，两者读取时序相同。
module spectrum_twiddle_rom #(parameter USE_EG_BRAM=1)(
    input wire clk,input wire [8:0] addr,output wire signed [17:0] q
);
    function [17:0] coeff;
        input [8:0] index;
        begin
        case(index)
        9'd0: coeff=18'h1ffff;
        9'd1: coeff=18'h1fff6;
        9'd2: coeff=18'h1ffd9;
        9'd3: coeff=18'h1ffa7;
        9'd4: coeff=18'h1ff62;
        9'd5: coeff=18'h1ff09;
        9'd6: coeff=18'h1fe9d;
        9'd7: coeff=18'h1fe1d;
        9'd8: coeff=18'h1fd89;
        9'd9: coeff=18'h1fce1;
        9'd10: coeff=18'h1fc26;
        9'd11: coeff=18'h1fb58;
        9'd12: coeff=18'h1fa75;
        9'd13: coeff=18'h1f980;
        9'd14: coeff=18'h1f876;
        9'd15: coeff=18'h1f75a;
        9'd16: coeff=18'h1f629;
        9'd17: coeff=18'h1f4e6;
        9'd18: coeff=18'h1f38f;
        9'd19: coeff=18'h1f225;
        9'd20: coeff=18'h1f0a8;
        9'd21: coeff=18'h1ef18;
        9'd22: coeff=18'h1ed74;
        9'd23: coeff=18'h1ebbe;
        9'd24: coeff=18'h1e9f4;
        9'd25: coeff=18'h1e818;
        9'd26: coeff=18'h1e629;
        9'd27: coeff=18'h1e427;
        9'd28: coeff=18'h1e212;
        9'd29: coeff=18'h1dfeb;
        9'd30: coeff=18'h1ddb1;
        9'd31: coeff=18'h1db65;
        9'd32: coeff=18'h1d907;
        9'd33: coeff=18'h1d696;
        9'd34: coeff=18'h1d413;
        9'd35: coeff=18'h1d17e;
        9'd36: coeff=18'h1ced8;
        9'd37: coeff=18'h1cc1f;
        9'd38: coeff=18'h1c955;
        9'd39: coeff=18'h1c679;
        9'd40: coeff=18'h1c38b;
        9'd41: coeff=18'h1c08c;
        9'd42: coeff=18'h1bd7c;
        9'd43: coeff=18'h1ba5b;
        9'd44: coeff=18'h1b728;
        9'd45: coeff=18'h1b3e5;
        9'd46: coeff=18'h1b091;
        9'd47: coeff=18'h1ad2c;
        9'd48: coeff=18'h1a9b6;
        9'd49: coeff=18'h1a631;
        9'd50: coeff=18'h1a29a;
        9'd51: coeff=18'h19ef4;
        9'd52: coeff=18'h19b3e;
        9'd53: coeff=18'h19778;
        9'd54: coeff=18'h193a2;
        9'd55: coeff=18'h18fbd;
        9'd56: coeff=18'h18bc8;
        9'd57: coeff=18'h187c4;
        9'd58: coeff=18'h183b1;
        9'd59: coeff=18'h17f8f;
        9'd60: coeff=18'h17b5e;
        9'd61: coeff=18'h1771e;
        9'd62: coeff=18'h172d1;
        9'd63: coeff=18'h16e74;
        9'd64: coeff=18'h16a0a;
        9'd65: coeff=18'h16592;
        9'd66: coeff=18'h1610b;
        9'd67: coeff=18'h15c78;
        9'd68: coeff=18'h157d7;
        9'd69: coeff=18'h15328;
        9'd70: coeff=18'h14e6d;
        9'd71: coeff=18'h149a4;
        9'd72: coeff=18'h144cf;
        9'd73: coeff=18'h13fee;
        9'd74: coeff=18'h13b00;
        9'd75: coeff=18'h13606;
        9'd76: coeff=18'h130ff;
        9'd77: coeff=18'h12bee;
        9'd78: coeff=18'h126d0;
        9'd79: coeff=18'h121a8;
        9'd80: coeff=18'h11c74;
        9'd81: coeff=18'h11735;
        9'd82: coeff=18'h111eb;
        9'd83: coeff=18'h10c97;
        9'd84: coeff=18'h10738;
        9'd85: coeff=18'h101d0;
        9'd86: coeff=18'h0fc5d;
        9'd87: coeff=18'h0f6e1;
        9'd88: coeff=18'h0f15b;
        9'd89: coeff=18'h0ebcc;
        9'd90: coeff=18'h0e633;
        9'd91: coeff=18'h0e092;
        9'd92: coeff=18'h0dae9;
        9'd93: coeff=18'h0d536;
        9'd94: coeff=18'h0cf7c;
        9'd95: coeff=18'h0c9b9;
        9'd96: coeff=18'h0c3ef;
        9'd97: coeff=18'h0be1d;
        9'd98: coeff=18'h0b844;
        9'd99: coeff=18'h0b264;
        9'd100: coeff=18'h0ac7d;
        9'd101: coeff=18'h0a68f;
        9'd102: coeff=18'h0a09b;
        9'd103: coeff=18'h09aa1;
        9'd104: coeff=18'h094a0;
        9'd105: coeff=18'h08e9a;
        9'd106: coeff=18'h0888f;
        9'd107: coeff=18'h0827e;
        9'd108: coeff=18'h07c68;
        9'd109: coeff=18'h0764d;
        9'd110: coeff=18'h0702e;
        9'd111: coeff=18'h06a0b;
        9'd112: coeff=18'h063e3;
        9'd113: coeff=18'h05db7;
        9'd114: coeff=18'h05788;
        9'd115: coeff=18'h05156;
        9'd116: coeff=18'h04b20;
        9'd117: coeff=18'h044e8;
        9'd118: coeff=18'h03ead;
        9'd119: coeff=18'h0386f;
        9'd120: coeff=18'h0322f;
        9'd121: coeff=18'h02bee;
        9'd122: coeff=18'h025aa;
        9'd123: coeff=18'h01f65;
        9'd124: coeff=18'h0191f;
        9'd125: coeff=18'h012d8;
        9'd126: coeff=18'h00c91;
        9'd127: coeff=18'h00648;
        9'd128: coeff=18'h00000;
        9'd129: coeff=18'h3f9b8;
        9'd130: coeff=18'h3f36f;
        9'd131: coeff=18'h3ed28;
        9'd132: coeff=18'h3e6e1;
        9'd133: coeff=18'h3e09b;
        9'd134: coeff=18'h3da56;
        9'd135: coeff=18'h3d412;
        9'd136: coeff=18'h3cdd1;
        9'd137: coeff=18'h3c791;
        9'd138: coeff=18'h3c153;
        9'd139: coeff=18'h3bb18;
        9'd140: coeff=18'h3b4e0;
        9'd141: coeff=18'h3aeaa;
        9'd142: coeff=18'h3a878;
        9'd143: coeff=18'h3a249;
        9'd144: coeff=18'h39c1d;
        9'd145: coeff=18'h395f5;
        9'd146: coeff=18'h38fd2;
        9'd147: coeff=18'h389b3;
        9'd148: coeff=18'h38398;
        9'd149: coeff=18'h37d82;
        9'd150: coeff=18'h37771;
        9'd151: coeff=18'h37166;
        9'd152: coeff=18'h36b60;
        9'd153: coeff=18'h3655f;
        9'd154: coeff=18'h35f65;
        9'd155: coeff=18'h35971;
        9'd156: coeff=18'h35383;
        9'd157: coeff=18'h34d9c;
        9'd158: coeff=18'h347bc;
        9'd159: coeff=18'h341e3;
        9'd160: coeff=18'h33c11;
        9'd161: coeff=18'h33647;
        9'd162: coeff=18'h33084;
        9'd163: coeff=18'h32aca;
        9'd164: coeff=18'h32517;
        9'd165: coeff=18'h31f6e;
        9'd166: coeff=18'h319cd;
        9'd167: coeff=18'h31434;
        9'd168: coeff=18'h30ea5;
        9'd169: coeff=18'h3091f;
        9'd170: coeff=18'h303a3;
        9'd171: coeff=18'h2fe30;
        9'd172: coeff=18'h2f8c8;
        9'd173: coeff=18'h2f369;
        9'd174: coeff=18'h2ee15;
        9'd175: coeff=18'h2e8cb;
        9'd176: coeff=18'h2e38c;
        9'd177: coeff=18'h2de58;
        9'd178: coeff=18'h2d930;
        9'd179: coeff=18'h2d412;
        9'd180: coeff=18'h2cf01;
        9'd181: coeff=18'h2c9fa;
        9'd182: coeff=18'h2c500;
        9'd183: coeff=18'h2c012;
        9'd184: coeff=18'h2bb31;
        9'd185: coeff=18'h2b65c;
        9'd186: coeff=18'h2b193;
        9'd187: coeff=18'h2acd8;
        9'd188: coeff=18'h2a829;
        9'd189: coeff=18'h2a388;
        9'd190: coeff=18'h29ef5;
        9'd191: coeff=18'h29a6e;
        9'd192: coeff=18'h295f6;
        9'd193: coeff=18'h2918c;
        9'd194: coeff=18'h28d2f;
        9'd195: coeff=18'h288e2;
        9'd196: coeff=18'h284a2;
        9'd197: coeff=18'h28071;
        9'd198: coeff=18'h27c4f;
        9'd199: coeff=18'h2783c;
        9'd200: coeff=18'h27438;
        9'd201: coeff=18'h27043;
        9'd202: coeff=18'h26c5e;
        9'd203: coeff=18'h26888;
        9'd204: coeff=18'h264c2;
        9'd205: coeff=18'h2610c;
        9'd206: coeff=18'h25d66;
        9'd207: coeff=18'h259cf;
        9'd208: coeff=18'h2564a;
        9'd209: coeff=18'h252d4;
        9'd210: coeff=18'h24f6f;
        9'd211: coeff=18'h24c1b;
        9'd212: coeff=18'h248d8;
        9'd213: coeff=18'h245a5;
        9'd214: coeff=18'h24284;
        9'd215: coeff=18'h23f74;
        9'd216: coeff=18'h23c75;
        9'd217: coeff=18'h23987;
        9'd218: coeff=18'h236ab;
        9'd219: coeff=18'h233e1;
        9'd220: coeff=18'h23128;
        9'd221: coeff=18'h22e82;
        9'd222: coeff=18'h22bed;
        9'd223: coeff=18'h2296a;
        9'd224: coeff=18'h226f9;
        9'd225: coeff=18'h2249b;
        9'd226: coeff=18'h2224f;
        9'd227: coeff=18'h22015;
        9'd228: coeff=18'h21dee;
        9'd229: coeff=18'h21bd9;
        9'd230: coeff=18'h219d7;
        9'd231: coeff=18'h217e8;
        9'd232: coeff=18'h2160c;
        9'd233: coeff=18'h21442;
        9'd234: coeff=18'h2128c;
        9'd235: coeff=18'h210e8;
        9'd236: coeff=18'h20f58;
        9'd237: coeff=18'h20ddb;
        9'd238: coeff=18'h20c71;
        9'd239: coeff=18'h20b1a;
        9'd240: coeff=18'h209d7;
        9'd241: coeff=18'h208a6;
        9'd242: coeff=18'h2078a;
        9'd243: coeff=18'h20680;
        9'd244: coeff=18'h2058b;
        9'd245: coeff=18'h204a8;
        9'd246: coeff=18'h203da;
        9'd247: coeff=18'h2031f;
        9'd248: coeff=18'h20277;
        9'd249: coeff=18'h201e3;
        9'd250: coeff=18'h20163;
        9'd251: coeff=18'h200f7;
        9'd252: coeff=18'h2009e;
        9'd253: coeff=18'h20059;
        9'd254: coeff=18'h20027;
        9'd255: coeff=18'h2000a;
        9'd256: coeff=18'h00000;
        9'd257: coeff=18'h00648;
        9'd258: coeff=18'h00c91;
        9'd259: coeff=18'h012d8;
        9'd260: coeff=18'h0191f;
        9'd261: coeff=18'h01f65;
        9'd262: coeff=18'h025aa;
        9'd263: coeff=18'h02bee;
        9'd264: coeff=18'h0322f;
        9'd265: coeff=18'h0386f;
        9'd266: coeff=18'h03ead;
        9'd267: coeff=18'h044e8;
        9'd268: coeff=18'h04b20;
        9'd269: coeff=18'h05156;
        9'd270: coeff=18'h05788;
        9'd271: coeff=18'h05db7;
        9'd272: coeff=18'h063e3;
        9'd273: coeff=18'h06a0b;
        9'd274: coeff=18'h0702e;
        9'd275: coeff=18'h0764d;
        9'd276: coeff=18'h07c68;
        9'd277: coeff=18'h0827e;
        9'd278: coeff=18'h0888f;
        9'd279: coeff=18'h08e9a;
        9'd280: coeff=18'h094a0;
        9'd281: coeff=18'h09aa1;
        9'd282: coeff=18'h0a09b;
        9'd283: coeff=18'h0a68f;
        9'd284: coeff=18'h0ac7d;
        9'd285: coeff=18'h0b264;
        9'd286: coeff=18'h0b844;
        9'd287: coeff=18'h0be1d;
        9'd288: coeff=18'h0c3ef;
        9'd289: coeff=18'h0c9b9;
        9'd290: coeff=18'h0cf7c;
        9'd291: coeff=18'h0d536;
        9'd292: coeff=18'h0dae9;
        9'd293: coeff=18'h0e092;
        9'd294: coeff=18'h0e633;
        9'd295: coeff=18'h0ebcc;
        9'd296: coeff=18'h0f15b;
        9'd297: coeff=18'h0f6e1;
        9'd298: coeff=18'h0fc5d;
        9'd299: coeff=18'h101d0;
        9'd300: coeff=18'h10738;
        9'd301: coeff=18'h10c97;
        9'd302: coeff=18'h111eb;
        9'd303: coeff=18'h11735;
        9'd304: coeff=18'h11c74;
        9'd305: coeff=18'h121a8;
        9'd306: coeff=18'h126d0;
        9'd307: coeff=18'h12bee;
        9'd308: coeff=18'h130ff;
        9'd309: coeff=18'h13606;
        9'd310: coeff=18'h13b00;
        9'd311: coeff=18'h13fee;
        9'd312: coeff=18'h144cf;
        9'd313: coeff=18'h149a4;
        9'd314: coeff=18'h14e6d;
        9'd315: coeff=18'h15328;
        9'd316: coeff=18'h157d7;
        9'd317: coeff=18'h15c78;
        9'd318: coeff=18'h1610b;
        9'd319: coeff=18'h16592;
        9'd320: coeff=18'h16a0a;
        9'd321: coeff=18'h16e74;
        9'd322: coeff=18'h172d1;
        9'd323: coeff=18'h1771e;
        9'd324: coeff=18'h17b5e;
        9'd325: coeff=18'h17f8f;
        9'd326: coeff=18'h183b1;
        9'd327: coeff=18'h187c4;
        9'd328: coeff=18'h18bc8;
        9'd329: coeff=18'h18fbd;
        9'd330: coeff=18'h193a2;
        9'd331: coeff=18'h19778;
        9'd332: coeff=18'h19b3e;
        9'd333: coeff=18'h19ef4;
        9'd334: coeff=18'h1a29a;
        9'd335: coeff=18'h1a631;
        9'd336: coeff=18'h1a9b6;
        9'd337: coeff=18'h1ad2c;
        9'd338: coeff=18'h1b091;
        9'd339: coeff=18'h1b3e5;
        9'd340: coeff=18'h1b728;
        9'd341: coeff=18'h1ba5b;
        9'd342: coeff=18'h1bd7c;
        9'd343: coeff=18'h1c08c;
        9'd344: coeff=18'h1c38b;
        9'd345: coeff=18'h1c679;
        9'd346: coeff=18'h1c955;
        9'd347: coeff=18'h1cc1f;
        9'd348: coeff=18'h1ced8;
        9'd349: coeff=18'h1d17e;
        9'd350: coeff=18'h1d413;
        9'd351: coeff=18'h1d696;
        9'd352: coeff=18'h1d907;
        9'd353: coeff=18'h1db65;
        9'd354: coeff=18'h1ddb1;
        9'd355: coeff=18'h1dfeb;
        9'd356: coeff=18'h1e212;
        9'd357: coeff=18'h1e427;
        9'd358: coeff=18'h1e629;
        9'd359: coeff=18'h1e818;
        9'd360: coeff=18'h1e9f4;
        9'd361: coeff=18'h1ebbe;
        9'd362: coeff=18'h1ed74;
        9'd363: coeff=18'h1ef18;
        9'd364: coeff=18'h1f0a8;
        9'd365: coeff=18'h1f225;
        9'd366: coeff=18'h1f38f;
        9'd367: coeff=18'h1f4e6;
        9'd368: coeff=18'h1f629;
        9'd369: coeff=18'h1f75a;
        9'd370: coeff=18'h1f876;
        9'd371: coeff=18'h1f980;
        9'd372: coeff=18'h1fa75;
        9'd373: coeff=18'h1fb58;
        9'd374: coeff=18'h1fc26;
        9'd375: coeff=18'h1fce1;
        9'd376: coeff=18'h1fd89;
        9'd377: coeff=18'h1fe1d;
        9'd378: coeff=18'h1fe9d;
        9'd379: coeff=18'h1ff09;
        9'd380: coeff=18'h1ff62;
        9'd381: coeff=18'h1ffa7;
        9'd382: coeff=18'h1ffd9;
        9'd383: coeff=18'h1fff6;
        9'd384: coeff=18'h1ffff;
        9'd385: coeff=18'h1fff6;
        9'd386: coeff=18'h1ffd9;
        9'd387: coeff=18'h1ffa7;
        9'd388: coeff=18'h1ff62;
        9'd389: coeff=18'h1ff09;
        9'd390: coeff=18'h1fe9d;
        9'd391: coeff=18'h1fe1d;
        9'd392: coeff=18'h1fd89;
        9'd393: coeff=18'h1fce1;
        9'd394: coeff=18'h1fc26;
        9'd395: coeff=18'h1fb58;
        9'd396: coeff=18'h1fa75;
        9'd397: coeff=18'h1f980;
        9'd398: coeff=18'h1f876;
        9'd399: coeff=18'h1f75a;
        9'd400: coeff=18'h1f629;
        9'd401: coeff=18'h1f4e6;
        9'd402: coeff=18'h1f38f;
        9'd403: coeff=18'h1f225;
        9'd404: coeff=18'h1f0a8;
        9'd405: coeff=18'h1ef18;
        9'd406: coeff=18'h1ed74;
        9'd407: coeff=18'h1ebbe;
        9'd408: coeff=18'h1e9f4;
        9'd409: coeff=18'h1e818;
        9'd410: coeff=18'h1e629;
        9'd411: coeff=18'h1e427;
        9'd412: coeff=18'h1e212;
        9'd413: coeff=18'h1dfeb;
        9'd414: coeff=18'h1ddb1;
        9'd415: coeff=18'h1db65;
        9'd416: coeff=18'h1d907;
        9'd417: coeff=18'h1d696;
        9'd418: coeff=18'h1d413;
        9'd419: coeff=18'h1d17e;
        9'd420: coeff=18'h1ced8;
        9'd421: coeff=18'h1cc1f;
        9'd422: coeff=18'h1c955;
        9'd423: coeff=18'h1c679;
        9'd424: coeff=18'h1c38b;
        9'd425: coeff=18'h1c08c;
        9'd426: coeff=18'h1bd7c;
        9'd427: coeff=18'h1ba5b;
        9'd428: coeff=18'h1b728;
        9'd429: coeff=18'h1b3e5;
        9'd430: coeff=18'h1b091;
        9'd431: coeff=18'h1ad2c;
        9'd432: coeff=18'h1a9b6;
        9'd433: coeff=18'h1a631;
        9'd434: coeff=18'h1a29a;
        9'd435: coeff=18'h19ef4;
        9'd436: coeff=18'h19b3e;
        9'd437: coeff=18'h19778;
        9'd438: coeff=18'h193a2;
        9'd439: coeff=18'h18fbd;
        9'd440: coeff=18'h18bc8;
        9'd441: coeff=18'h187c4;
        9'd442: coeff=18'h183b1;
        9'd443: coeff=18'h17f8f;
        9'd444: coeff=18'h17b5e;
        9'd445: coeff=18'h1771e;
        9'd446: coeff=18'h172d1;
        9'd447: coeff=18'h16e74;
        9'd448: coeff=18'h16a0a;
        9'd449: coeff=18'h16592;
        9'd450: coeff=18'h1610b;
        9'd451: coeff=18'h15c78;
        9'd452: coeff=18'h157d7;
        9'd453: coeff=18'h15328;
        9'd454: coeff=18'h14e6d;
        9'd455: coeff=18'h149a4;
        9'd456: coeff=18'h144cf;
        9'd457: coeff=18'h13fee;
        9'd458: coeff=18'h13b00;
        9'd459: coeff=18'h13606;
        9'd460: coeff=18'h130ff;
        9'd461: coeff=18'h12bee;
        9'd462: coeff=18'h126d0;
        9'd463: coeff=18'h121a8;
        9'd464: coeff=18'h11c74;
        9'd465: coeff=18'h11735;
        9'd466: coeff=18'h111eb;
        9'd467: coeff=18'h10c97;
        9'd468: coeff=18'h10738;
        9'd469: coeff=18'h101d0;
        9'd470: coeff=18'h0fc5d;
        9'd471: coeff=18'h0f6e1;
        9'd472: coeff=18'h0f15b;
        9'd473: coeff=18'h0ebcc;
        9'd474: coeff=18'h0e633;
        9'd475: coeff=18'h0e092;
        9'd476: coeff=18'h0dae9;
        9'd477: coeff=18'h0d536;
        9'd478: coeff=18'h0cf7c;
        9'd479: coeff=18'h0c9b9;
        9'd480: coeff=18'h0c3ef;
        9'd481: coeff=18'h0be1d;
        9'd482: coeff=18'h0b844;
        9'd483: coeff=18'h0b264;
        9'd484: coeff=18'h0ac7d;
        9'd485: coeff=18'h0a68f;
        9'd486: coeff=18'h0a09b;
        9'd487: coeff=18'h09aa1;
        9'd488: coeff=18'h094a0;
        9'd489: coeff=18'h08e9a;
        9'd490: coeff=18'h0888f;
        9'd491: coeff=18'h0827e;
        9'd492: coeff=18'h07c68;
        9'd493: coeff=18'h0764d;
        9'd494: coeff=18'h0702e;
        9'd495: coeff=18'h06a0b;
        9'd496: coeff=18'h063e3;
        9'd497: coeff=18'h05db7;
        9'd498: coeff=18'h05788;
        9'd499: coeff=18'h05156;
        9'd500: coeff=18'h04b20;
        9'd501: coeff=18'h044e8;
        9'd502: coeff=18'h03ead;
        9'd503: coeff=18'h0386f;
        9'd504: coeff=18'h0322f;
        9'd505: coeff=18'h02bee;
        9'd506: coeff=18'h025aa;
        9'd507: coeff=18'h01f65;
        9'd508: coeff=18'h0191f;
        9'd509: coeff=18'h012d8;
        9'd510: coeff=18'h00c91;
        9'd511: coeff=18'h00648;
        default:coeff=18'd0;
        endcase
        end
    endfunction
    generate if(USE_EG_BRAM) begin : anlogic
        EG_PHY_BRAM #(.MODE("PDPW8K"),.DATA_WIDTH_A("18"),.DATA_WIDTH_B("18"),
            .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.RESETMODE("SYNC"),
            .CEAMUX("0"),.CEBMUX("1"),.OCEAMUX("1"),.OCEBMUX("1"),
            .RSTAMUX("0"),.RSTBMUX("0"),.WEAMUX("0"),.WEBMUX("0"),
            .CSA0("1"),.CSA1("1"),.CSA2("1"),.CSB0("1"),.CSB1("1"),.CSB2("1"),
            .INIT_00(256'hfb5afc76fc80fd75fd58fe26fee1fe89ff1dff9dff09ff62ffa7ffd9fff6ffff),
            .INIT_01(256'hed65eeb1efebf112f227f329f418f4f4f5bef674f718f8a8f925f98ffae6fb29),
            .INIT_02(256'hd62cd891d9e5db28dd5bde7ce08ce18be379e455e61fe7d8e87eea13eb96ec07),
            .INIT_03(256'hb774b9d1bb1ebd5ebf8fc1b1c3c4c5c8c7bdc9a2cb78cd3ecff4d19ad331d4b6),
            .INIT_04(256'h90a893d095ee98ff9b069d009feea2cfa4a4a76da928abd7ae78b00bb292b50a),
            .INIT_05(256'h64b9677c6a366de97092733375cc785b7be17e5d80d08338869788eb8b358e74),
            .INIT_06(256'h350b382e3b4d3e68417e448f479a4aa04da1509b538f567d59645c445f1d61ef),
            .INIT_07(256'h0348069109d80c1f0f6512aa15ee192f1c6f1fad22e8252028562b882eb731e3),
            .INIT_08(256'hd149d478d7aadae0dd18e053e391e6d1ea12ed56f09bf3e1f628f96ffcb80000),
            .INIT_09(256'ha0e3a3bca69ca983ac71af65b25fb560b866bb71be82c198c4b3c7d2caf5ce1d),
            .INIT_0A(256'h74cb771579697cc87f3081a3841f87a58a348ccd8f6e921795ca98849b479e11),
            .INIT_0B(256'h4d6e4ff55188542956d858935b5c5d316012620064fa67016a126c306f58718c),
            .INIT_0C(256'h2ccf2e66300c32c23488365e38433a383c3c3e4f407142a244e2462f488c4af6),
            .INIT_0D(256'h146a15ed1782182819e11bab1c871e751f74218422a524d8261b276f29d42b4a),
            .INIT_0E(256'h051a067106db075808e8098c0a420b0c0be80cd70dd90eee1015114f129b13f9),
            .INIT_0F(256'h000a00270059009e00f7006300e30177011f01da02a8028b0380038a04a604d7),
            .INIT_10(256'h2eb72b882856252022e81fad1c6f192f15ee12aa0f650c1f09d8069103480000),
            .INIT_11(256'h5f1d5c445964567d538f509b4da14aa0479a448f417e3e683b4d382e350b31e3),
            .INIT_12(256'h8b3588eb8697833880d07e5d7be1785b75cc733370926de96a36677c64b961ef),
            .INIT_13(256'hb292b00bae78abd7a928a76da4a4a2cf9fee9d009b0698ff95ee93d090a88e74),
            .INIT_14(256'hd331d19acff4cd3ecb78c9a2c7bdc5c8c3c4c1b1bf8fbd5ebb1eb9d1b774b50a),
            .INIT_15(256'heb96ea13e87ee7d8e61fe455e379e18be08cde7cdd5bdb28d9e5d891d62cd4b6),
            .INIT_16(256'hfae6f98ff925f8a8f718f674f5bef4f4f418f329f227f112efebeeb1ed65ec07),
            .INIT_17(256'hfff6ffd9ffa7ff62ff09ff9dff1dfe89fee1fe26fd58fd75fc80fc76fb5afb29),
            .INIT_18(256'hfb5afc76fc80fd75fd58fe26fee1fe89ff1dff9dff09ff62ffa7ffd9fff6ffff),
            .INIT_19(256'hed65eeb1efebf112f227f329f418f4f4f5bef674f718f8a8f925f98ffae6fb29),
            .INIT_1A(256'hd62cd891d9e5db28dd5bde7ce08ce18be379e455e61fe7d8e87eea13eb96ec07),
            .INIT_1B(256'hb774b9d1bb1ebd5ebf8fc1b1c3c4c5c8c7bdc9a2cb78cd3ecff4d19ad331d4b6),
            .INIT_1C(256'h90a893d095ee98ff9b069d009feea2cfa4a4a76da928abd7ae78b00bb292b50a),
            .INIT_1D(256'h64b9677c6a366de97092733375cc785b7be17e5d80d08338869788eb8b358e74),
            .INIT_1E(256'h350b382e3b4d3e68417e448f479a4aa04da1509b538f567d59645c445f1d61ef),
            .INIT_1F(256'h0348069109d80c1f0f6512aa15ee192f1c6f1fad22e8252028562b882eb731e3),
            .INITP_00(256'h0154015500000001540505144414451405555501451110415401541044410555),
            .INITP_01(256'haabefbbbefaaffaafbeeeebafeaaaabfaebbbfbbaebebfaafffffffeaaffaafc),
            .INITP_02(256'h5541044410550055041111450155554051445044514140550000000155005500),
            .INITP_03(256'h0154015500000001540505144414451405555501451110415401541044410555)) u_rom(
            .clka(clk),.cea(1'b0),.wea(1'b0),.addra(13'd0),.dia(9'd0),.dib(9'd0),
            .csa(3'b111),.rsta(1'b0),.ocea(1'b1),
            .clkb(clk),.ceb(1'b1),.web(1'b0),.addrb({addr,4'b1111}),
            .csb(3'b111),.rstb(1'b0),.oceb(1'b1),.doa(q[8:0]),.dob(q[17:9]));
    end else begin : portable
        reg [17:0] q_reg;
        always @(posedge clk) q_reg<=coeff(addr);
        assign q=q_reg;
    end endgenerate
endmodule
`default_nettype wire

var display = {
  w:800, 
  h:600,
  ready:0,
  buf_img:null,
  buf_data:null,
};

handleCmdInput = function(e) {
  if(!e) e = window.event;
  value = document.getElementById("input_cmd").value;
  if((e.keyCode || e.which) == 13){
    writeToScreen('SENT: ' +  value);
    websocket.send(value);
  }
};

rgbImageData = function(x, y, vx, vy, width, height, arr, offset) {
  var img, i, j, data;
  img = cxt.createImageData(width, height);
  data = img.data;
  for (i=0, j=offset; i < (width * height * 4); i=i+4, j=j+3) {
    data[i    ] = arr[j    ];
    data[i + 1] = arr[j + 1];
    data[i + 2] = arr[j + 2];
    data[i + 3] = 255; // Set Alpha
  }
  cxt.putImageData(img, x - vx, y - vy);
};

rgbFill = function(x, y, width, height, rgb) {
  var img, i, j, data;
  img = cxt.createImageData(width, height);
  data = img.data;
  for (i=0, j=0; i < (width * height * 4); i=i+4, j=j+3) {
    data[i    ] = rgb[0]; //arr[j    ];
    data[i + 1] = rgb[1];  //arr[j + 1];
    data[i + 2] = rgb[2];  //arr[j + 2];
    data[i + 3] = 128; // Set Alpha
  }
  cxt.putImageData(img, x, y);
};

var pseudoFill = function() {
  var img, i,data;
  var width = display.w
  var height = display.h;
  var rgb={};
  img = display.buf_img;
  data = display.buf_data;
  for (i=0;i < (width * height * 4); i=i+4) {
    var gray = (i/4%width)&0xFF;
    // data[i    ] = lut[gray*3]; //arr[j    ];
    // data[i + 1] = lut[gray*3+1]; //arr[j + 1];
    // data[i + 2] = lut[gray*3+2];  //arr[j + 2];
    // data[i    ] = gray; //arr[j    ];
    // data[i + 1] = gray; //arr[j + 1];
    // data[i + 2] = gray;  //arr[j + 2];
    //rainbow(gray,rgb);
    myRainbow(gray,rgb);
    data[i    ] = rgb.r; //arr[j    ];
    data[i + 1] = rgb.g; //arr[j + 1];
    data[i + 2] = rgb.b;  //arr[j + 2];
    data[i + 3] = gray; // Set Alpha
  }
  cxt.putImageData(img, 0, 0);
};

var pseudo_color = [
function(data,width, height,arr){
  for (var i=0, j=0; j < width * height; i=i+4, j++) {
    var gray = arr[j];
    data[i    ] = gray;
    data[i + 1] = gray
    data[i + 2] = gray
    data[i + 3] = 255;
  }
},
function(data,width, height,arr){
  for (var i=0, j=0; j < width * height; i=i+4, j++) {
    var gray = arr[j];
    data[i    ] = gray;
    if(gray < 128){
      data[i + 1] = gray * 2;
    }else{
      data[i + 1] = (255 - gray)*2;
    }
    data[i + 2] = 255-gray + 10;
    data[i + 3] =  gray;
  }
},
function(data,width, height,arr){
  for (var i=0, j=0; j < width * height; i=i+4, j++) {
    var gray = arr[j];
    data[i    ] = lut[gray*3];
    data[i + 1] = lut[gray*3 + 1];
    data[i + 2] = lut[gray*3 + 2];
    data[i + 3] = gray;
  }
},
function(data,width, height,arr){
  var rgb={};
  for (var i=0, j=0; j < width * height; i=i+4, j++) {
    var gray =arr[j];
    //statistics(gray);
    //gray = colormap(gray,50,200,5,245);
    rainbow(gray,rgb);
    data[i    ] = rgb.r; //arr[j    ];
    data[i + 1] = rgb.g; //arr[j + 1];
    data[i + 2] = rgb.b;  //arr[j + 2];
    data[i + 3] = gray; // Set Alpha
  }
},
];

var gray_statistics = null;

var statistics = function(gray){
  if(gray_statistics==null){
    gray_statistics = new Array(300/60)
    for(var i=0;i<300/60;i++){
      gray_statistics[i] = 0;
    }
  }
  for(var i=1;i<=300/60;i++){
    if(gray<=60*i) {
      gray_statistics[i-1]++;
      break;
    }
  }
};

grayImageData = function(x, y, width, height, arr) {
  var i, j, data,gray;
  if(display.buf_data == null) return;
  data = display.buf_data;
  pseudo_color[0](data,width,height,arr);
  cxt.putImageData(display.buf_img, x, y);
};

resizeScreen = function(w,h){
  world=document.getElementById("world");
  world.width = w;
  world.height = h;
  
  cxt=world.getContext("2d");

  display.w = w;
  display.h = h;
  display.buf_img = cxt.createImageData(w, h);
  display.buf_data = display.buf_img.data;

  var grd=cxt.createLinearGradient(0,0,display.w,display.h);
  grd.addColorStop(0,"#FF0000");
  grd.addColorStop(1,"#00FF00");
  cxt.fillStyle=grd;
  cxt.fillRect(0,0,display.w,display.h);

  cxt.font="30px Verdana";
    // Create gradient
    var gradient=cxt.createLinearGradient(0,0,world.width,0);
    gradient.addColorStop("0","magenta");
    gradient.addColorStop("0.5","blue");
    gradient.addColorStop("1.0","red");
  // Fill with gradient
  cxt.fillStyle=gradient;
  cxt.fillText("LS300 V2.0",300,300);
};

initScreen = function(){
  resizeScreen(800,600);
};

updateScreen = function(data){
  if(display.ready)
   grayImageData(0,0,display.w,display.h,data);
};

writeToScreen = function(message) {
  var div = document.createElement('div');
  div.innerHTML = message;
  var out = document.getElementById('output');
  out.appendChild(div);

  while (out.children.length > 2) {
    out.firstChild.remove();
  }
};

window.onload = function() {
  var url = 'ws://' + window.location.host + '/foo';
  websocket = new WebSocket(url);
  websocket.binaryType = 'arraybuffer';
  websocket.onopen = function(ev) {
    writeToScreen('CONNECTED');
    initScreen();
  };
  websocket.onclose = function(ev) {
    writeToScreen('DISCONNECTED');
  };
  websocket.onmessage = function(ev) {
    Processor.process(ev.data);
  };
  websocket.onerror = function(ev) {
    writeToScreen('<span style="color: red; ">ERROR: </span> ' + ev.data);
  };

  lut = gen_pseudo_color_lut(3);
};

var Processor={
  connect: function(){websocket.send("connect");},
  disconnect: function(){websocket.send("disconnect");},
  size:function(){websocket.send("size");},
  pointscan: function(){websocket.send("pointscan");},
  photoscan: function(){websocket.send("photoscan");},
  cancel: function(){Timer.stop(); websocket.send("cancel");},
  config: function(){websocket.send("config");},
  update: function(){
    websocket.send("data");
  },
  response: function(str,rp,code){
    if(code && rp.query == "start"){
      console.log("start");
    }
    writeToScreen('<span style="color: blue;">' + str + '</span>');
  },
  process:function(evdata){
    if(evdata instanceof ArrayBuffer){
      var bytearray =new Uint8Array(evdata);
      updateScreen(bytearray);
      return null;
    }else{
     var retstr="";
      //{query:%s,\nret:[{code:0,msg:message},{}]}
      //{query:size,\nret:{code:1,size:{w:600,h:800}}}
      try{
        var rp = jsl.parser.parse(evdata);
        if(rp){
          var code = 1;
          if(rp.query == "size" && rp.ret.size){
           display.ready = 1;
           resizeScreen(rp.ret.size.w,rp.ret.size.h);
           Timer.start(Processor.update,10);
           retstr += 'resize: ' + rp.ret.size.w + 'x' +rp.ret.size.h;
         }else{
          retstr += 'Command&nbsp;' + rp.query + ':<br>&nbsp;stack:';
          for(var i=0;i<rp.ret.length;i++){
            if(rp.ret[i].code != null) {
              retstr += '<br>&nbsp;&nbsp;code:' + rp.ret[i].code + '&nbsp;msg:' + rp.ret[i].msg;
              code = code && (rp.ret[i].code>0); 
            }
          }
        }
      }
    } catch (exc) {
      retstr += 'RESPONSE: ' + evdata;
    }
    Processor.response(retstr,rp,code);
  }
},
};

var Timer={
  start:function(func,delay)
  {

    if(Timer.get_data_timer) Timer.stop();
    Timer.get_data_timer = window.setInterval(
      function()
      {
        if(func())
        {
          Timer.stop();
        };
      },
      delay
      );
  },
  stop:function()
  {
   window.clearInterval(Timer.get_data_timer);
 },
};

var gen_pseudo_color_lut=function(bit_per_channle){
  var bits_total = 3*bit_per_channle;
  var color_num = 1 << bits_total;
  var lut=new Array(3*color_num);
  var r,g,b;

  for(var c = 0; c < color_num; c++){
    r = g = b = 0;
    for(var k = 0; k < bits_total; ){
      b = (b << 1) + ((c >> k++) & 1);
      g = (g << 1) + ((c >> k++) & 1);
      r = (r << 1) + ((c >> k++) & 1);
    }
    r = r << (8 - bit_per_channle);
    g = g << (8 - bit_per_channle);
    b = b << (8 - bit_per_channle);

    lut[3*c] = r; lut[3*c+1] = g; lut[3*c+2] = b;
  }

  return lut;
}


//     R    G    B   gray
//----------------------------------
// 红 255,   0,   0   255
// 橙 255, 127,   0   204
// 黄 255, 255,   0   153
// 绿   0, 255,   0   102
// 青   0, 255, 255    51
// 蓝   0,   0, 255     0
var  rainbow = function(gray,rgb)
{
  var r=gray,g=gray,b=gray;
  if (gray <= 51)
  {
    b = 255;
    g = gray*5;
    r = 0;
  }
  else if (gray <= 102)
  {
    gray-=51;
    b = 255-gray*5;
    g = 255;
    r = 0;
  }
  else if (gray <= 153)
  {
    gray-=102;
    b = 0;
    g = 255;
    r = gray*5;
  }
  else if (gray <= 204)
  {
    gray-=153;
    b = 0;
    g = 255-(128.0*gray/51.0+0.5)&0xFF;
    r = 255;
  }
  else
  {
    gray-=204;
    b = 0;
    g = 127-(127.0*gray/51.0+0.5)&0xFF;
    r = 255;
  }
  rgb.r=r;
  rgb.g=g;
  rgb.b=b; 
};

//当x<x1：      f(x) = y1/x1*x;
//当x1<=x<=x2： f(x) = (y2-y1)/(x2-x1)*(x-x1)+y1;
//当x>x2：      f(x) = (255-y2)/(255-x2)*(x-x2)+y2;    
//其中x1,y1,x2,y2是图中ac,bd两个转折点的坐标
var colormap=function(gray,x1,x2,y1,y2){
  if(gray<x1)
    return y1/x1*gray;
  else if(gray<x2)
    return (y2-y1)/(x2-x1)*(gray-x1)+y1;
  else
    return (255-y2)/(255-x2)*(gray-x2)+y2; 
};


var myRainbow = function(gray,rgb)
{
  var Level = 255;
  var Extent = 256/4;
  var r,g,b;

  var flag = gray/Extent;
  if(flag<1)
  {
    r = 0;
    g = 0;
    b = 4*gray;
  }
  else if(flag<2)
  {
    r = 0;
    g = 4*gray-Level;
    b = -4*gray+2*Level;

  }
  else if(flag<3)
  {
    r = 4*gray-2*Level;
    g = Level;
    b = 0;
  }
  else if(flag<=4)
  {
    r = Level;
    g = -4*gray+4*Level;
    b = 0;

  }

  rgb.r=r;
  rgb.g=g;
  rgb.b=b; 
}
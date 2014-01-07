var srcx = 0; //起始点水平相对距离
var srcy = 0; //起始点垂直相对距离
var desx = 0; //终点水平相对距离
var desy = 0; //重点垂直相对距离
var anglehstart = 0; //水平角度范围起点
var anglehend = 0; //水平角度范围终点
var anglevstart = 0; //垂直角度范围起点
var anglevend = 0; //垂直角度范围终点
var tmpsrcx = 0; //起点水平绝对距离
var tmpsrcy = 0; //起点垂直绝对距离
var tmpdesx = 0; //终点水平绝对距离
var tmpdesy = 0; //终点垂直绝对距离
var width = screen.width; //屏幕宽度
var height = screen.height; //屏幕高度
var last_config = {
    s_v : -45,   e_v : 90,   
    s_h : 0,     e_h : 360
};
var photowidth = 0; //图片原始宽度
var photoheight = 0; //图片原始高度
var pic_update_delay = 5000;
var device_state_update_delay = 2000;
var messagebox;
var jcrop;

///////////////////////////////////////////状态初始化/////////////////////////////////
//初始化页面

$(window).load(
    function(){
        pre_log("window.onload");
    }
);

function pre_log(msg){
    var d = new Date();
   console.log(d.toLocaleTimeString() + ':' + d.getMilliseconds()+ ' ' + msg);
}

function log(msg) {
    $("#messagebox").val($("#messagebox").val() + new Date().toLocaleTimeString() + ' ' + msg + '\n');
    $("#messagebox").animate({
        scrollTop: $("#messagebox")[0].scrollHeight - $("#messagebox").height()
    },
    500);
}

var err_timer = null;
var c_flag = 0;
function reset_errmsg(){
    // $("#errmsg").css({color:'white'});
    // $("#errmsg").text("LS300 V2.0");
    if(err_timer != null){
        c_flag = 0;
        clearTimeout(err_timer);
        $("#errmsg").slideUp();
    }
}
function flashmsg(){
    if(c_flag == 0)
         $("#errmsg").slideDown();
    if(c_flag%2 == 0)
        $("#errmsg").css({color:'yellow'});
    else
        $("#errmsg").css({color:'white'});
    c_flag++;
    if(c_flag >= 7)
        reset_errmsg();
    else
        err_timer = setTimeout("flashmsg()",1000);
}

function showmsg(msg){
    reset_errmsg();
    $("#errmsg").text("Error:\t" + msg);
    err_timer = setTimeout("flashmsg()",300);
}

function start_monitor() {
    update_state();
    update_photo();
    log("Monitor started.");
}

function update_state() {
    //log("update state");
    getdevicestate();
    getbattery();
    getangle();
    gettilt();
    getled();
    gettemperature();
    setTimeout("update_state()", device_state_update_delay);
}

function update_photo() {
    //log("update image");
    if ($("#state").val() == "STATE_WORK") {
        getgray();
    }
    setTimeout("update_photo()", pic_update_delay);
}

function getgraysize() //获取预览图像大小
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=graysize',
        async: false,
        success: function(ev) {
            photowidth = ev.width;
            photoheight = ev.height;
            log('photo:' + photowidth + 'x' + photoheight);
        }
    });
}
function getgray() {
    getgraysize();
    jcrop.setImage(base_url + "?q=gray.jpg&timestamp=" + new Date().getTime());
}

function getdevicestate() //获取设备状态
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=devicestate',
        success: function(ev) {
            $("#state").html(ev.state);
        }
    });
}
function getbattery() //获取电池电量
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=battery',
        success: function(ev) {
            $("#battery").html(ev.battery);
        }
    });
}
function getangle() //获取水平角度
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=angle',
        success: function(ev) {
            $("#angle").html(ev.angle);
        }
    });
}
function gettilt() //获取水平倾斜角度
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=tilt',
        success: function(ev) {
            $("#tilt").html(ev.dx + " " + ev.dy);
        }
    });
}
function gettemperature() //获取仪器温度
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=temperature',
        success: function(ev) {
            $("#temperature").html(ev.temperature);
        }
    });
}
function getavailableplusdelay() //返回所有水平速度列表
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=availablePlusDelay',
        success: function(ev) {
            var select = $("#plusdelay");
            for (var i = 0; i < ev.availablePlusDelay.length; i++) {
                select.append("<option value='" + ev.availablePlusDelay[i] + "'>" + ev.availablePlusDelay[i] + "</option>")
            }
        }
    });
}
function getavailableResolution() //返回所有可用垂直分辨率列表
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=availableResolution',
        success: function(ev) {
            var select = $("#resolution");
            for (var i = 0; i < ev.availableResolution.length; i++) {
                select.append("<option value='" + ev.availableResolution[i] + "'>" + ev.availableResolution[i] + "</option>")
            }
        }
    });
}
function getavailableFrequency() //返回所有垂直频率列表
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=availableFrequency',
        success: function(ev) {
            var select = $("#frequency");
            for (var i = 0; i < ev.availableFrequency.length; i++) {
                select.append("<option value='" + ev.availableFrequency[i] + "'>" + ev.availableFrequency[i] + "</option>")
            }
        }
    });
}
function getavailablePrecision() //返回所有推荐精度列表
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=availablePrecision',
        success: function(ev) {
            var select = $("#precision");
            for (var i = 0; i < ev.availablePrecision.length; i++) {
                select.append("<option value='" + ev.availablePrecision[i].frequency + "," + ev.availablePrecision[i].resolution + "," + ev.availablePrecision[i].plusdelay + "'>" + ev.availablePrecision[i].name + "</option>")
            }
        }
    });
}
function getled() //获取led状态
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=led',
        success: function(ev) {
            $("#led").html(ev.led);
        }
    });
}
function setled(color) //获取led状态
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=led&color=' + color,
        success: function(ev) {
            log(ev.message);
            showmsg(ev.message);
        }
    });
}
function turntable(angle) //转台操作
{
    if(angle == 0) return;
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=turntable&angle=' + angle,
        success: function(ev) {
            log(ev.message);
            showmsg(ev.message);
        }
    });
}
function turntable_searchzero(){
     $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=searchzero',
        success: function(ev) {
            log(ev.message);
            showmsg(ev.message);
        }
    });
}

function setconfig(plusdelay, frequency, resolution, start_angle_h, end_angle_h, start_angle_v, end_angle_v) //配置扫描参数
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=config&plusdelay=' + plusdelay + '&frequency=' + frequency + '&resolution=' + resolution + '&start_angle_h=' + start_angle_h + '&end_angle_h=' + end_angle_h + '&start_angle_v=' + start_angle_v + '&end_angle_v=' + end_angle_v,
        success: function(ev) {
            if(ev.success != 0){
                last_config.s_h = start_angle_h;
                last_config.e_h = end_angle_h;
                last_config.s_v = start_angle_v;
                last_config.e_v = end_angle_v;
            }
            log(ev.message);
            showmsg(ev.message);
        }
    });
}
function startpointscan() //启动点云扫描
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=pointscan',
        success: function(ev) {
            log(ev.message);showmsg(ev.message);
        }
    });
}
function startphotoscan() //启动全景拍照
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=photoscan',
        success: function(ev) {
            log(ev.message);showmsg(ev.message);
        }
    });
}

function takephoto() //拍摄照片
{
    $.ajax({
        dataType: 'jsonp',
        url: base_url + '?q=takephoto',
        success: function(ev) {
            log(ev.message);showmsg(ev.message);
        }
    });
}
function checkconfig() {
    if ($("#resolution").val() == "") {
        return false;
    }
    if ($("#frequency").val() == "") {
        return false;
    }
    if ($("#plusdelay").val() == "") {
        return false;
    }
    if ($("#anglehstart").val() == "") {
        return false;
    }
    if ($("#anglehend").val() == "") {
        return false;
    }
    if ($("#anglevstart").val() == "") {
        return false;
    }
    if ($("#anglevend").val() == "") {
        return false;
    }
    return true;
}
function setupdate_delay() {
    if ($("#videodelay").val() != "") {
        pic_update_delay = $("#videodelay").val()*1000;
    }
    if ($("#statedelay").val() != "") {
        device_state_update_delay = $("#statedelay").val()*1000;
    }
    return true;
}

function shortten(num) {
    return parseFloat(num.toPrecision(3));
}

function update_select_area(coords){
    if(parseInt(coords.w) > 0){
        anglehstart = last_config.s_h + coords.x * 
            (last_config.e_h - last_config.s_h) / photowidth;
        anglehend = last_config.s_h + coords.x2 * 
            (last_config.e_h - last_config.s_h) / photowidth;
        anglevstart = (photoheight - coords.y2) * 
        (last_config.e_v - last_config.s_v) / photoheight + last_config.s_v;
        anglevend = (photoheight -coords.y) * 
        (last_config.e_v - last_config.s_v) / photoheight + last_config.s_v;

        $("#anglehstart").val(shortten(anglehstart));
        $("#anglehend").val(shortten(anglehend));
        $("#anglevstart").val(shortten(anglevstart));
        $("#anglevend").val(shortten(anglevend));
    }
}

function globle_init() {
    ///////////////////////////////////////////下拉框初始化/////////////////////////////////
    getavailableplusdelay();
    getavailableResolution();
    getavailableFrequency();
    getavailablePrecision();
    ///////////////////////////////////////////事件初始化/////////////////////////////////
    //推荐下拉框赋值事件
    $("#precision").change(function() {
        var selection = $(this).children('option:selected').val();
        var tmp = selection.split(",");
        $("#frequency").val(tmp[0]);
        $("#resolution").val(tmp[1]);
        $("#plusdelay").val(tmp[2]);
    })
    //扫描点云事件
    $("#pointscan").click(function() {
        if (checkconfig()) {
            setconfig($("#plusdelay").val(), $("#frequency").val(), $("#resolution").val(), $("#anglehstart").val(), $("#anglehend").val(), $("#anglevstart").val(), $("#anglevend").val());
            startpointscan();
        } else {
            showmsg("配置不能为空");
        }
    });
    //扫描照片事件
    $("#photoscan").click(function() {
        if (checkconfig()) {
            setconfig($("#plusdelay").val(), $("#frequency").val(), $("#resolution").val(), $("#anglehstart").val(), $("#anglehend").val(), $("#anglevstart").val(), $("#anglevend").val());
            startphotoscan();
        } else {
            showmsg("配置不能为空");
        }
    });
    //放弃当前任务
    function abortmission() {
        $.ajax({
            dataType: 'jsonp',
            url: base_url + '?q=cancel',
            success: function(ev) {
                log(ev.message);showmsg(ev.message);
            }
        });
    }

    //初始化按钮事件
    $("#chushihua").click(function() {
        setconfig(50, 5, 0.5, 0, 360, -45, 90);
        startpointscan(); //初始化图片大小 服务返回大小均为0
    });
    //取消操作
    $("#cancel").click(function() {
        abortmission();
    });
    //拍照事件
    $("#takephoto").click(function() {
        takephoto();
    });
    //执行控制LED按钮事件
    $(".radioled").change(function() {
        var color = $("input[name=led]:checked").val();
        setled(color);
    });
    //执行控制转台角度按钮事件
    $("#turnleft").click(function() {
        var angle = 0 - $("#angletext").val();
        turntable(angle);
    });
    $("#turnright").click(function() {
        var angle = $("#angletext").val();
        turntable(angle);
    });
    $("#turnzero").click(function() {
        turntable_searchzero();
    });
    //清空消息记录
    $("#clear").click(function() {
        $("#messagebox").val("日志：\n");
    });

    $.ajaxSetup ({
            cache: false //关闭AJAX相应的缓存
    });
}

function selectarea(){
    if($('#selectarea').text() != 'Confirm'){
        $('#selectarea').text('Confirm');
        jcrop.setImage(base_url + "?q=gray.jpg&timestamp=" + new Date().getTime());
    }else{
        $('#selectarea').text('Select');
    }
}

var running = 3;
function ready_hook() {
    if (running != 1) {
        running--;
        return;
    }
    $("#messagebox").val("日志：\n");
    log("window.ready_hook");
    $("#image").Jcrop({
        onChange:update_select_area,
        onSelect:update_select_area,
        onDblClick:function(){this.release();},
        aspectRatio:0,
        boxWidth:$('#img_div').width(),
        addClass:'image'
    },function(){
        jcrop = this;
        getgray();
    }); 
    globle_init();
    // start_monitor();
}

$(function() {
    pre_log("window.ready");
    $("#view").attr('src', base_url + 'ae/ae.html');
    $("#shell").attr('src', base_url + 'shellinabox/terminal_e.html');
    $("#head").load('head_e.html', ready_hook);
    $("#scan").load('scan_e.html', ready_hook);
    $("#setting").load('setting_e.html', ready_hook);
});
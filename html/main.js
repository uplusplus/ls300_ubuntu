// This file is part of Mongoose project, http://code.google.com/p/mongoose

var chat = {
  // Backend URL, string.
  // 'http://backend.address.com' or '' if backend is the same as frontend
  backendUrl: '',
  maxVisibleMessages: 10,
  errorMessageFadeOutTimeoutMs: 2000,
  errorMessageFadeOutTimer: null,
  lastMessageId: 0,
  getMessagesIntervalMs: 1000,
};

chat.normalizeText = function(text) {
  return text; //text.replace('<', '&lt;').replace('>', '&gt;');
};

chat.refresh = function(data) {

  if (data === undefined) {
    return;
  }

  $.each(data, function(index, entry) {
    var row = $('<div>').addClass('message-row').appendTo('#mml');
    var timestamp = (new Date(entry.timestamp * 1000)).toLocaleTimeString();
    $('<span>')
      .addClass('message-timestamp')
      .html('[' + timestamp + ']')
      .prependTo(row);
    $('<span>')
      .addClass('message-user')
      .addClass(entry.user ? '' : 'message-user-server')
      .html(chat.normalizeText((entry.user || '[server]') + ':'))
      .appendTo(row);
    $('<span>')
      .addClass('message-text')
      .addClass(entry.user ? '' : 'message-text-server')
      .html(chat.normalizeText(entry.text))
      .appendTo(row);
    chat.lastMessageId = Math.max(chat.lastMessageId, entry.id) + 1;
  });

  // Keep only chat.maxVisibleMessages, delete older ones.
  while ($('#mml').children().length > chat.maxVisibleMessages) {
    $('#mml div:first-child').remove();
  }
};

chat.getMessages = function(enter_loop) {
  $.ajax({
    dataType: 'jsonp',
    url: chat.backendUrl + '/ajax/get_messages',
    data: {last_id: chat.lastMessageId},
    success: chat.refresh,
    error: function() {
    },
  });
  if (enter_loop) {
    window.setTimeout('chat.getMessages(true)', chat.getMessagesIntervalMs);
  }
};

chat.handleMenuItemClick = function(ev) {
  $('.menu-item').removeClass('menu-item-selected');  // Deselect menu buttons
  $(this).addClass('menu-item-selected');  // Select clicked button
  $('.main').addClass('hidden');  // Hide all main windows
  $('#' + $(this).attr('name')).removeClass('hidden');  // Show main window
};

chat.showError = function(message) {
  $('#error').html(message).fadeIn('fast');
  window.clearTimeout(chat.errorMessageFadeOutTimer);
  chat.errorMessageFadeOutTimer = window.setTimeout(function() {
      $('#error').fadeOut('slow');
  }, chat.errorMessageFadeOutTimeoutMs);
};

chat.handleMessageInput = function(ev) {
  var input = ev.target;
  if (ev.keyCode != 13 || !input.value)
    return;
  //input.disabled = true;
  $.ajax({
    dataType: 'jsonp',
    url: chat.backendUrl + '/ajax/send_message',
    data: {text: input.value},
    success: function(ev) {
      input.value = '';
      input.disabled = false;
      chat.getMessages(false);
    },
    error: function(ev) {
      chat.showError('Error sending message');
      input.disabled = false;
    },
  });
};

chat.sendMessage = function(msg){
 $.ajax({
    dataType: 'jsonp',
    url: chat.backendUrl + '/ajax/send_message',
    data: {text: msg},
    success: function(ev) {
      chat.getMessages(false);
    },
    error: function(ev) {
      chat.showError('Error sending message');
      input.disabled = false;
    },
  });
};

$(document).ready(function() {
  $('.menu-item').click(chat.handleMenuItemClick);
  $('.message-input').keypress(chat.handleMessageInput);
  $('#gray').width(document.body.clientWidth);
  $('#gray').height(document.body.clientHeight/2);
  chat.getMessages(true);
});

var lasttime = new Date().getTime();

var Processor={
  connect: function(){ chat.sendMessage("connect"); },
  disconnect: function(){ chat.sendMessage("disconnect");},
  size:function(){ chat.sendMessage("size");},
  pointscan: function(){ chat.sendMessage("pointscan");},
  photoscan: function(){ chat.sendMessage("photoscan");},
  cancel: function(){Timer.stop();  chat.sendMessage("cancel");},
  config: function(){ chat.sendMessage("config");},
  update: function(){
    Timer.start(startnextimage,2000);
    lasttime = new Date().getTime();
    console.log("start update");
  }
 }


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

function indof(x,arr)
{
	if (arr.indexOf)
		return arr.indexOf(x);
	for (var i = 0; i<arr.length; i++)
	{
		if (arr[i] == x)
			return i;
	}
	return -1;
}
function indoff(x,arr,start)
{
	if (arr.indexOf)
		return arr.indexOf(x,start);
	for (var i = start; i<arr.length; i++)
	{
		if (arr[i] == x)
			return i;
	}
	return -1;
}

function getCookie(c_name)
{
if (document.cookie.length>0)
  {
  c_start=indof(c_name+"=",document.cookie);
  if (c_start!=-1)
    {
    c_start=c_start + c_name.length+1;
    c_end=indoff(";",document.cookie,c_start);
    if (c_end==-1) c_end=document.cookie.length;
    return unescape(document.cookie.substring(c_start,c_end));
    }
  }
	return "";
}

function startnextimage()
{
	var now = new Date();
	var addr = 'gray.jpg?'+now.getTime();
	$("#gray")
		.unbind()
		.load(function(){
			console.log(addr+" load");
			var ctime = getCookie("time");
			if (ctime >= lasttime)
			{
				$('#gray').attr('src', addr);//$(this).attr('src'));
				lasttime = ctime;
			}

		})
		.error(function(){
		})
		.attr('src',addr);
}


// vim:ts=2:sw=2:et

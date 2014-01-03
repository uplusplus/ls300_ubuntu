// var base_url='http://192.168.10.154:8082/'; //
var base_url='/';
/*第一种形式 第二种形式 更换显示样式*/ 
function setTab(name,cursel,n){ 
	for(i=1;i<=n;i++){ 
		var menu=document.getElementById(name+i); 
		var con=document.getElementById("con_"+name+"_"+i); 
		if(i==cursel){
 			menu.className="btn active";
			con.style.display="block"; 
			menu.background="white";
			
		}else{
			menu.className="btn";
			con.style.display="none"; 
		}
	} 
} 
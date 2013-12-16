HTTP/1.0 200 OK
Content-Type: text/html

<html>
  <p>Prime numbers from 0 to 100, calculated by Lua:</p>
 <?
    function is_prime(n)
      if n <= 0 then return false end
      if n <= 2 then return true end
      if (n % 2 == 0) then return false end
      for i = 3, n / 2, 2 do
        if (n % i == 0) then return false end
      end
      return true
    end

    for i = 1, 100 do
      if is_prime(i) then mg.write('<span>' .. i .. '</span>&nbsp;') end
    end

 ?>

 <p>Reading POST data from Lua (click submit):</p>
 <form method="POST" ><input type="text" name="t1"/><input type="submit"></form>

 <pre>
   POST data: [<?
      local post_data = ''
      if mg.request_info.request_method == 'POST' then
        post_data = mg.read()
      end
      mg.write(post_data)
    ?>]
   request method: [<? mg.write(mg.request_info.request_method) ?>]
   IP/port: [<? mg.write(mg.request_info.remote_ip, ':',
                         mg.request_info.remote_port) ?>]
   URI: [<? mg.write(mg.request_info.uri) ?>]
   HTTP version [<? mg.write(mg.request_info.http_version) ?>]
   HEADERS:
   <?
     for name, value in pairs(mg.request_info.http_headers) do
       mg.write(name, ':', value, '\n')
     end
   ?>
</pre>

<p>URI:</p> 
<?
--使用Lua遍历指定目录，获取所有文件，并使用自定义的函数处理每一个文件
--遍历目录，并对所有的文件进行处理
function get_dir_file(dirpath,func)
    --os.execute("dir " .. dirpath .. " /s > temp.txt")
    os.execute('ls "' .. dirpath .. '"/*/ -d  > temp.txt')
    io.input("temp.txt")
    local dirname = ""
    local filename = ""
    for line in io.lines() do
        local a,b,c
        --匹配目录
        a,b,c=string.find(line,"^%s*(.+)%s+")
        if a then
         dirname = c
         --print(c)
     end
     --匹配文件
        a,b,c=string.find(line,"^%d%d%d%d%-%d%d%-%d%d%s-%d%d:%d%d%s-[%d%,]+%s+(.+)%s-$")
        if a then
         filename = c
         --print(c)
         func(dirname .. "\\" .. filename)
        end
     --print(line)
    end
end
--获取指定的最后一个字符的位置
function get_last_word(all,word)
    local b = 0
    local last = nil
    while true do
        local s,e = string.find(all, word, b) -- find 'next' word
        if s == nil then
         break
        else
         last = s
        end
         b = s + string.len(word)
    end
    return last
end
get_dir_file(".",mg_write)
?>
</html>

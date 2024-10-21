#include <iostream>
#include <thread>//多线程
#include <chrono>//时间相关
#include <functional>//函数包装器  
//std::function<void(const std::string&,const std::string&)> 名字 //参数为两个字符串，返回值为void
#include "cpp-httplib/httplib.h"//下载相关

class Download
{
private:
    /* data */
public:
    void download(const std::string& host, const std::string& path,
    const std::function<void(const std::string&,const std::string&)> & callback_word_count)//函数包装器
    {
        std::cout<<"线程"<<std::this_thread::get_id()<<std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200)
        {
            callback_word_count(path, response->body);
        }
    };

    void start_download(const std::string &host, const std::string &path, 
    const std::function<void(const std::string &,const std::string &)>&callback_word_count)//函数包装器
    {
        auto down_func = std::bind(&Download::download, this, std::placeholders::_1, 
        std::placeholders::_2, std::placeholders::_3);
        std::thread thread(down_func, host, path, callback_word_count);
        thread.detach();//使用 thread.detach() 使线程与主线程分离，这样主线程结束时不必等待这个下载线程结束。
    };
};

int main()
{
    auto d = Download();
    auto word_count=[](const std::string& path, const std::string& result)-> void
    {
        std::cout<<"下载完成"<<path<<":"<<result.length()<<result.substr(0,9)<<std::endl;

    };
    d.download("http://0.0.0.0:8000", "/novel1.txt", word_count);
    d.download("http://0.0.0.0:8000", "/novel2.txt", word_count);
    d.download("http://0.0.0.0:8000", "/novel3.txt", word_count);

    std::this_thread::sleep_for(std::chrono::seconds(10));//等待10秒
}

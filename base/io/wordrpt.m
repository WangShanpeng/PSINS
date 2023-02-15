function wordrpt(var1, var2, var3)
% 采用Matlab自动生成word报告. Refs:
%   https://zhuanlan.zhihu.com/p/350652763
%   https://blog.csdn.net/weixin_47919042/article/details/120156511
%   https://wenku.baidu.com/view/955424cd561810a6f524ccbff121dd36a32dc4d3.html?_wkts_=1670851263863&bdQuery=matlab+Shape.AddPicture
%   基于Matlab的word报告自动生成方法研究
%
% Prototype: wordrpt(var1, var2, var3)
% Inputs: var1, var2, var3 - input parameters, please see the code
%
% Example:
% wordrpt(0); wordrpt([800;600])
% for k=1:5
%     wordrpt(sprintf('第%d节标题', k), k);
%     wordrpt('	Say something...');
%     myfig, plot(randn(10,1)); grid on 
%     wordrpt(gcf, 'xxx'); close(gcf);
% end
% wordrpt(-1);

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/12/2022
global ActxWord Doc figno figwh
    if isempty(ActxWord)
        try     % 如果Word服务器已打开，则直接返回其句柄
            ActxWord = actxGetRunningServer('Word.Application');  ActxWord.Visible = 1;
        catch   % 创建Word服务器
            ActxWord = actxserver('Word.Application'); 
        end
    end
    if isnumeric(var1)
        if var1(1)==0,
            Doc=ActxWord.Documents.Add;  Doc.Content.Start=0;              %%%% wordrpt(0, '报告名', '作者');   新建word文件
            Doc.PageSetup.TopMargin=60;  Doc.PageSetup.bottomMargin=50; 
            Doc.PageSetup.LeftMargin=50; Doc.PageSetup.RightMargin=50; 
            if nargin<2, var2=('PSINS数据分析报告'); end;
            ActxWord.Selection.TypeParagraph;
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.Text = var2;
            ActxWord.Selection.ParagraphFormat.Alignment = 'wdAlignParagraphCenter';  ActxWord.Selection.Font.Size = 20; ActxWord.Selection.Font.Bold = 1;
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.TypeParagraph;
            if nargin<3, str=datestr(now,31); var3=(['by PSINS, ',str]); end;
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.Text = var3;
            ActxWord.Selection.ParagraphFormat.Alignment = 'wdAlignParagraphCenter';  ActxWord.Selection.Font.Size = 12;
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.TypeParagraph; ActxWord.Selection.TypeParagraph;
            figno = 1; figwh = [800; 600];
        elseif var1(1)==-1,
            if nargin<2, str=datestr(now,30); var2=sprintf('PSINS数据分析%s.docx',str(end-5:end)); end
            if isempty(strfind(var2,'.docx')), var2=[var2,'.docx']; end
            Doc.SaveAs2(var2); % Doc.Close; ActxWord.Quit(); ActxWord=[];  %%%% wordrpt(-1, '文件名.docx');  保存word文件
        elseif length(var1)==2
            figwh = var1;                                                  %%%% wordrpt([800;600]);  设置图片大小           
        else
            ActxWord.Selection.Start = Doc.Content.end;
            set(var1, 'Position', [50 50 figwh(1) figwh(2)]);
            print(var1, '-dmeta'); %为去除图周围空白：Win开始->设置->系统->屏幕->缩放与布局->100%
            ActxWord.Selection.Range.Paste;                                %%%% wordrpt(gcf, '图标题');  插入figure图
%             print(gcf, '-djpeg', 'psins_tmp.jpg');    Doc.Shapes.AddPicture('psins_tmp.jpg');
            ActxWord.Selection.ParagraphFormat.Alignment = 'wdAlignParagraphCenter';
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.TypeParagraph;
            ActxWord.Selection.Start = Doc.Content.end;   % 图自动编号
            str=sprintf('图%d', figno); figno=figno+1; if nargin==2, str=[str,'  ',var2]; end
            ActxWord.Selection.Text = str;
            ActxWord.Selection.ParagraphFormat.Alignment = 'wdAlignParagraphCenter';  ActxWord.Selection.Font.Bold = 1;
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.TypeParagraph;
        end
    else
        if nargin==2
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.paragraphs.OutlinePromote;
            ActxWord.Selection.Text = sprintf('%.1f %s', var2, var1);      %%%% wordrpt(txt, 1.1);  写标题，标题编号1.1
            ActxWord.Selection.Font.Size = 16;
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.TypeParagraph;
        else                                   
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.Text = var1;                                %%%% wordrpt(txt);  写文字
            ActxWord.Selection.Font.Size = 12;
            ActxWord.Selection.Start = Doc.Content.end;
            ActxWord.Selection.TypeParagraph;
        end
    end
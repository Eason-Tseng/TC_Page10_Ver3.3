ifndef CFG
  CFG :=Release_Build  #預設Release模式
endif
ifeq ($(CFG),Debug_Build)
  BUILD_DIR :=linux/Debug_Build#路徑
  CFLAGS = -g2 -O0 -MD -I/usr/lib -I/usr/lib64  #編譯參數
endif
ifeq ($(CFG),Release_Build)
  BUILD_DIR :=linux/Release_Build#路徑
  CFLAGS = -march=i486  #編譯參數
endif
ifneq ($(CFG),Debug_Build)
  ifneq ($(CFG),Release_Build)
    $(error An incorrect configuration was specified)
  endif
endif

NAME = TC_Page10_Ver3.3  #執行檔名稱
SOURCE_cpp = $(wildcard *.cpp)  #表示所有.cpp檔案
SOURCE_o = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SOURCE_cpp))  #將所有.cpp檔更換成.o檔（表示所有.o檔案）
	
CXX = g++   # gcc or g++
LDFLAGS = -lrt -lpthread  # gcc 參數
	
all:builder_dir Linker

builder_dir:
	-mkdir -p $(BUILD_DIR)

Linker:$(SOURCE_o)
	$(CXX) -o $(BUILD_DIR)/$(NAME) $^ $(LDFLAGS)
 
.PHONY:clean  # 避免目錄下有檔案與"clean"同名
clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.d $(BUILD_DIR)/$(NAME)
	ls $(BUILD_DIR)
 
$(BUILD_DIR)/%.o:%.cpp # compiler
	$(CXX) -c -o $@ -Wall $(CFLAGS) $<
 
# $^ 所有依賴項目的檔名,並以空格隔開這些檔名.(這份清單已經拿掉重複的檔名)
# $< 當前目標項目的依賴項目
# $@ 當前目標項目名稱
# $? 當前目標項目需重建的依賴項目
# %  Makefile目標項目與依賴項目的萬用字元,相當於linux下的*
#
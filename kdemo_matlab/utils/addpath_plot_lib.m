function addpath_plot_lib(MAIN_PATH)

    addpath([MAIN_PATH '/plot_lib/']);
	
    % add dependencies from other libraries

    addpath_argValidation_lib(MAIN_PATH);

end
